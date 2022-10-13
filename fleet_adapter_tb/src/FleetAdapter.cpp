/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "FleetAdapter.hpp"

#include <rmf_fleet_adapter/agv/parse_graph.hpp>

#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/agv/Interpolate.hpp>

//==============================================================================
FleetAdapter::FleetAdapter(const rclcpp::NodeOptions& options)
: Node("turtlebot_fleet_adapter", options)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Starting %s",
    this->get_name()
  );

  _data = std::make_shared<Data>();

  _data->fleet_name = this->declare_parameter(
    "fleet_name", "turtlebot");
  RCLCPP_INFO(
    this->get_logger(),
    "Configuring fleet [%s].",\
    _data->fleet_name.c_str()
  );

  auto traits = rmf_traffic::agv::VehicleTraits{
    {0.3, 0.7},
    {1.5, 2.0},
    rmf_traffic::Profile{
      rmf_traffic::geometry::make_final_convex<
        rmf_traffic::geometry::Circle>(0.15),
      rmf_traffic::geometry::make_final_convex<
        rmf_traffic::geometry::Circle>(0.15)
    }
  };
  traits.get_differential()->set_reversible(true);

  std::string navgraph_path = this->declare_parameter("navgraph_path", "");
  auto graph =
    rmf_fleet_adapter::agv::parse_graph(navgraph_path, traits);

  const auto battery_opt = rmf_battery::agv::BatterySystem::make(
    24.0, 30.0, 2.0);

  if (!battery_opt.has_value())
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Invalid battery parameters");
  }
  const auto battery_system =
    std::make_shared<rmf_battery::agv::BatterySystem>(*battery_opt);

  const auto mechanical_opt = rmf_battery::agv::MechanicalSystem::make(
    10.0, 10.0, 0.2);
  if (!mechanical_opt.has_value())
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Invalid mechanical parameters");
  }

  std::shared_ptr<rmf_battery::agv::SimpleMotionPowerSink> motion_sink =
    std::make_shared<rmf_battery::agv::SimpleMotionPowerSink>(
    *battery_system, mechanical_opt.value());

  const auto ambient_power_system = rmf_battery::agv::PowerSystem::make(
    30.0);
  if (!ambient_power_system)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Invalid values supplied for ambient power system");
  }
  std::shared_ptr<rmf_battery::agv::SimpleDevicePowerSink> ambient_sink =
    std::make_shared<rmf_battery::agv::SimpleDevicePowerSink>(
    *battery_system, *ambient_power_system);

  auto tool_power_system = rmf_battery::agv::PowerSystem::make(
    0.0);
  if (!tool_power_system)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Invalid values supplied for tool power system");
  }
  std::shared_ptr<rmf_battery::agv::SimpleDevicePowerSink> tool_sink =
    std::make_shared<rmf_battery::agv::SimpleDevicePowerSink>(
    *battery_system, *tool_power_system);

  std::vector<std::string> actions = {"teleop"};
  actions = this->declare_parameter("actions", actions);

  const std::string server_uri_string =
    this->declare_parameter("server_uri", std::string());

  std::optional<std::string> server_uri = std::nullopt;
  if (!server_uri_string.empty())
    server_uri = server_uri_string;

  _data->config = std::make_shared<Configuration>(
    _data->fleet_name,
    std::move(traits),
    std::move(graph),
    battery_system,
    motion_sink,
    ambient_sink,
    tool_sink,
    0.1,
    1.0,
    true,
    actions,
    nullptr,
    server_uri
  );

  _data->adapter = EasyFullControl::make(
    *_data->config
  );

  if (_data->adapter != nullptr)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Created EasyFullControl adapter instance."
    );

    _data->adapter->start();
    add_robots();
  }
  else
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Failed to create EasyFullControl adapter instance."
    );
  }
}
//==============================================================================
void FleetAdapter::add_robots()
{

  auto add_robot =
  [data = _data](
    const std::string& name,
    const std::string& ns,
    const std::string& initial_map_name,
    rclcpp::Node::SharedPtr node)
  {
    auto insertion = data->robots.insert({name, nullptr});
    if (!insertion.second)
    {
      RCLCPP_WARN(
        node->get_logger(),
        "Attempted to add robot with name [%s] more than once. Ignoring...",
        name.c_str()
      );
      return;
    }
    auto robot = std::make_shared<Robot>();
    // This is a block call.
    RCLCPP_INFO(
      node->get_logger(),
      "Initializing robot [%s]", name.c_str()
    );
    if (robot->initialize(name, ns, initial_map_name, node))
    {
      insertion.first->second = std::move(robot);
      data->adapter->add_robot(
        insertion.first->second->get_state(),
        insertion.first->second->get_state,
        insertion.first->second->navigate,
        insertion.first->second->stop,
        insertion.first->second->dock,
        insertion.first->second->action_executor
      );
    }
    else
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Failed to initialize robot [%s]", name.c_str()
      );
    }
  };

  // We pass this classes node and not adapter node so that callback queues
  // can execute independently.
  auto node = shared_from_this();
  std::vector<std::string> robots = {"tb4"};
  robots = this->declare_parameter("robots", robots);
  std::vector<std::string> namespaces;
  std::vector<std::string> initial_map_names;
  for (const auto& r : robots)
  {
    namespaces.push_back(this->declare_parameter(r + ".namespace", ""));
    initial_map_names.push_back(this->declare_parameter(
      r + ".initial_map_name", "L1"));
  }

  // Spin separate threads to add each robot.
  for (std::size_t i = 0; i < robots.size(), ++i)
  {
    _threads.push_back(std::thread(
      add_robot, robots[i], namespaces[i], initial_map_names[i], node));
  }
}

//==============================================================================
auto FleetAdapter::Robot::initialize(
  const std::string& name,
  const std::string& ns,
  const std::string& initial_map_name,
  rclcpp::Node::SharedPtr node) -> bool
{
  return false;
}

//==============================================================================
auto FleetAdapter::Robot::get_state() -> RobotState
{
  auto state = RobotState(
    name,
    charger_name,
    map_name,
    location,
    battery_soc
  );

  return state;
}

//==============================================================================
auto FleetAdapter::Robot::navigate(
  const std::string& map_name,
  const Eigen::Vector3d goal,
  RobotUpdateHandlePtr robot_handle) -> GoalCompletedCallback
{
  auto cb =
  [this](
    rmf_traffic::Duration& remaining_time_,
    bool& request_replan_) -> bool
  {
    if (this->remaining_time.has_value())
    {
      // Robot is still navigating;
      remaining_time_ = this->remaining_time.value();
      return false;
    }
    return true;
  };

  // TODO(YV): Send nav goal.
  return cb;
}

//==============================================================================
auto FleetAdapter::Robot::stop() -> bool
{
  return true;
}

//==============================================================================
auto FleetAdapter::Robot::dock(
  const std::string& dock_name,
  RobotUpdateHandlePtr robot_handle) -> GoalCompletedCallback
{
  auto cb =
  [this](
    rmf_traffic::Duration& remaining_time_,
    bool& request_replan_) -> bool
  {
    if (this->remaining_time.has_value())
    {
      // Robot is still navigating;
      remaining_time_ = this->remaining_time.value();
      return false;
    }
    return true;
  };

  return cb;
}

//==============================================================================
void FleetAdapter::Robot::action_executor(
  const std::string& category,
  const nlohmann::json& description,
  RobotUpdateHandle::ActionExecution execution)
{
  execution.finished();
}

//==============================================================================
FleetAdapter::~FleetAdapter()
{
  for (auto& thread : _data->threads)
  {
    if (thread.joinable())
      thread.join();
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(FleetAdapter)

//==============================================================================
// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);
//   auto adapter = std::make_shared<FleetAdapter>();
//   rclcpp::spin(adapter);
//   rclcpp::shutdown();
//   return 0;
// }
