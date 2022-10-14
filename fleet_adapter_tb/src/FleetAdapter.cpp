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

#include <rmf_traffic_ros2/Time.hpp>

#include <tf2/LinearMath/Quaternion.h>

//==============================================================================
FleetAdapter::FleetAdapter(const rclcpp::NodeOptions& options)
{
  _data = std::make_shared<Data>();
  _data->node =
    std::make_shared<rclcpp::Node>("turtlebot_fleet_adapter", options);
  _data->threads.push_back(std::thread(
      [n = _data->node]()
      {
        while (rclcpp::ok())
        {
          rclcpp::spin_some(n);
        }
      }
  ));

  RCLCPP_INFO(
    _data->node->get_logger(),
    "Starting %s",
    _data->node->get_name()
  );

  _data->fleet_name = _data->node->declare_parameter(
    "fleet_name", "turtlebot");
  RCLCPP_INFO(
    _data->node->get_logger(),
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
        rmf_traffic::geometry::Circle>(0.2)
    }
  };
  traits.get_differential()->set_reversible(true);

  std::string navgraph_path =
    _data->node->declare_parameter("navgraph_path", "");
  auto graph =
    rmf_fleet_adapter::agv::parse_graph(navgraph_path, traits);

  const auto battery_opt = rmf_battery::agv::BatterySystem::make(
    24.0, 30.0, 2.0);

  if (!battery_opt.has_value())
  {
    RCLCPP_ERROR(
      _data->node->get_logger(),
      "Invalid battery parameters");
  }
  const auto battery_system =
    std::make_shared<rmf_battery::agv::BatterySystem>(*battery_opt);

  const auto mechanical_opt = rmf_battery::agv::MechanicalSystem::make(
    10.0, 10.0, 0.2);
  if (!mechanical_opt.has_value())
  {
    RCLCPP_ERROR(
      _data->node->get_logger(),
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
      _data->node->get_logger(),
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
      _data->node->get_logger(),
      "Invalid values supplied for tool power system");
  }
  std::shared_ptr<rmf_battery::agv::SimpleDevicePowerSink> tool_sink =
    std::make_shared<rmf_battery::agv::SimpleDevicePowerSink>(
    *battery_system, *tool_power_system);

  std::vector<std::string> actions = {"teleop"};
  actions = _data->node->declare_parameter("actions", actions);

  const std::string server_uri_string =
    _data->node->declare_parameter("server_uri", std::string());

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
      _data->node->get_logger(),
      "Created EasyFullControl adapter instance."
    );

    add_robots();
  }
  else
  {
    RCLCPP_INFO(
      _data->node->get_logger(),
      "Failed to create EasyFullControl adapter instance."
    );
  }
}
//==============================================================================
void FleetAdapter::add_robots()
{

  auto add_robot =
    [data = _data](
    const std::string& ns,
    const std::string& name,
    const std::string& charger_name,
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
      if (robot->initialize(ns, name, charger_name, initial_map_name, node))
      {
        insertion.first->second = std::move(robot);

        data->adapter->add_robot(
          insertion.first->second->get_state(),
          [robot = insertion.first->second]() -> RobotState
          {
            return robot->get_state();
          },
          [robot = insertion.first->second](
            const std::string& map_name,
            const Eigen::Vector3d goal,
            RobotUpdateHandlePtr robot_handle) -> GoalCompletedCallback
          {
            return robot->navigate(map_name, goal, robot_handle);
          },
          [robot = insertion.first->second]() -> bool
          {
            return robot->stop();
          },
          [robot = insertion.first->second](
            const std::string& dock_name,
            RobotUpdateHandlePtr robot_handle) -> GoalCompletedCallback
          {
            return robot->dock(dock_name, robot_handle);
          },
          [robot = insertion.first->second](
            const std::string& category,
            const nlohmann::json& description,
            RobotUpdateHandle::ActionExecution execution)
          {
            robot->action_executor(category, description, std::move(execution));
          }
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


  std::vector<std::string> robots = {"tb4"};
  robots = _data->node->declare_parameter("robots", robots);
  std::vector<std::string> namespaces;
  std::vector<std::string> charger_names;
  std::vector<std::string> initial_map_names;
  for (const auto& r : robots)
  {
    namespaces.push_back(_data->node->declare_parameter(r + ".namespace", ""));
    initial_map_names.push_back(_data->node->declare_parameter(
        r + ".initial_map_name", "L1"));
    charger_names.push_back(_data->node->declare_parameter(
        r + ".charger_waypoint", "tinyRobot1_charger"));
  }

  // Spin separate threads to add each robot.
  // We pass this classes node and not adapter node so that callback queues
  // can execute independently.
  for (std::size_t i = 0; i < robots.size(); ++i)
  {
    _data->threads.push_back(std::thread(
        add_robot, namespaces[i], robots[i], charger_names[i],
        initial_map_names[i], _data->node));
  }
}

//==============================================================================
auto FleetAdapter::Robot::initialize(
  const std::string& ns,
  const std::string& name_,
  const std::string& charger_name_,
  const std::string& initial_map_name_,
  rclcpp::Node::SharedPtr node_) -> bool
{
  name = name_;
  charger_name = charger_name_;
  map_name = initial_map_name_;
  node = node_;
  odom_sub = node->create_subscription<Odom>(
    ns + "/odom",
    rclcpp::QoS(10),
    [&](Odom::ConstSharedPtr msg)
    {
      tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      // TODO(YV): Transformation
      location = Eigen::Vector3d{
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        yaw};

      RCLCPP_DEBUG(
        node->get_logger(),
        "Robot [%s] is at [%.2f, %.2f, %.2f]",
        name.c_str(),
        location.value()[0],
        location.value()[1],
        location.value()[2]
      );
    }
  );

  // Wait for odom
  while (!location.has_value())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Wait for nav2 action server
  nav2_client = rclcpp_action::create_client<NavigationAction>(
    node,
    ns + "/navigate_to_pose");

  // TODO(YV): Add a timeout
  return nav2_client->wait_for_action_server();
}

//==============================================================================
auto FleetAdapter::Robot::get_state() -> RobotState
{
  const auto l =
    location.has_value() ? location.value() : Eigen::Vector3d{0, 0, 0};
  auto state = RobotState(
    name,
    charger_name,
    map_name,
    l,
    battery_soc
  );

  return state;
}

//==============================================================================
auto FleetAdapter::Robot::navigate(
  const std::string& map_name,
  const Eigen::Vector3d goal_,
  RobotUpdateHandlePtr robot_handle) -> GoalCompletedCallback
{
  auto cb =
    [this](
    rmf_traffic::Duration& remaining_time_,
    bool& request_replan_) -> bool
    {
      if (!finished_navigating)
      {
        // Robot is still navigating. Update remaining time once we receive a
        // feedback.
        if (remaining_time.has_value())
          remaining_time_ = remaining_time.value();
        return false;
      }
      return true;
    };

  auto goal = NavigationAction::Goal();
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = node->get_clock()->now();
  goal.pose.pose.position.x = goal_[0];
  goal.pose.pose.position.y = goal_[1];
  auto q = tf2::Quaternion();
  q.setEuler(goal_[2], 0.0, 0.0);
  goal.pose.pose.orientation = tf2::toMsg(q);

  auto goal_options =
    rclcpp_action::Client<NavigationAction>::SendGoalOptions();
  goal_options.goal_response_callback =
    [this](const GoalHandle::SharedPtr& goal_handle)
    {
      if (!goal_handle)
      {
        RCLCPP_ERROR(
          node->get_logger(),
          "Navigation goal was rejected by server for robot [%s]",
          name.c_str());
      }
      else
      {
        RCLCPP_INFO(
          node->get_logger(),
          "Navigation goal accepted by server for robot [%s].",
          name.c_str());
      }
    };
  goal_options.feedback_callback =
    [this](
    GoalHandle::SharedPtr,
    const std::shared_ptr<const NavigationAction::Feedback> feedback)
    {
      remaining_time =
        rmf_traffic_ros2::convert(feedback->estimated_time_remaining);
    };
  goal_options.result_callback =
    [this](const GoalHandle::WrappedResult& result)
    {
      switch (result.code)
      {
        case rclcpp_action::ResultCode::SUCCEEDED:
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(
            node->get_logger(), "Goal was aborted for robot %s.", name.c_str());
          return;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(
            node->get_logger(), "Goal was canceled for robot %s.",
            name.c_str());
          return;
        default:
          RCLCPP_ERROR(node->get_logger(), "Unknown result code");
          return;
      }
      finished_navigating = true;
      goal_handle = nullptr;
    };

  finished_navigating = false;
  remaining_time = std::nullopt;
  nav2_client->async_send_goal(goal, goal_options);
  return cb;
}

//==============================================================================
auto FleetAdapter::Robot::stop() -> bool
{
  if (goal_handle != nullptr)
  {
    nav2_client->async_cancel_goal(goal_handle);
  }
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
  for (const auto& [_, robot] : _data->robots)
  {
    robot->stop();
  }
  for (auto& thread : _data->threads)
  {
    if (thread.joinable())
      thread.join();
  }
}

//==============================================================================
void FleetAdapter::run()
{
  _data->adapter->wait();
}

// #include <rclcpp_components/register_node_macro.hpp>
// RCLCPP_COMPONENTS_REGISTER_NODE(FleetAdapter)

// ==============================================================================
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto adapter = std::make_shared<FleetAdapter>();
  adapter->run();
  rclcpp::shutdown();
  return 0;
}
