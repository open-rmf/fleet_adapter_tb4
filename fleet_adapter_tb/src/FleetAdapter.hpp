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

#ifndef SRC__FLEETADAPTER_HPP
#define SRC__FLEETADAPTER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <rmf_fleet_adapter/agv/EasyFullControl.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <memory>
#include <mutex>
#include <thread>

//==============================================================================
class FleetAdapter : public std::enable_shared_from_this<FleetAdapter>
{
public:
  using EasyFullControl = rmf_fleet_adapter::agv::EasyFullControl;
  using Graph = EasyFullControl::Graph;
  using VehicleTraits = EasyFullControl::VehicleTraits;
  using Configuration = EasyFullControl::Configuration;
  using RobotState = EasyFullControl::RobotState;
  using GetStateCallback = EasyFullControl::GetStateCallback;
  using GoalCompletedCallback = EasyFullControl::GoalCompletedCallback;
  using NavigationRequest = EasyFullControl::NavigationRequest;
  using RobotUpdateHandle = rmf_fleet_adapter::agv::RobotUpdateHandle;
  using RobotUpdateHandlePtr = rmf_fleet_adapter::agv::RobotUpdateHandlePtr;
  using StopRequest = EasyFullControl::StopRequest;
  using DockRequest = EasyFullControl::DockRequest;

  using NavigationAction = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigationAction>;
  using Odom = nav_msgs::msg::Odometry;

  /// Constructor
  FleetAdapter(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  void run();

  ~FleetAdapter();

private:
  struct Robot
  {
    std::string name;
    std::string charger_name;
    std::string map_name;
    std::optional<Eigen::Vector3d> location;
    double battery_soc = 1.0;
    std::optional<rmf_traffic::Duration> remaining_time;

    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<Odom>::SharedPtr odom_sub;
    rclcpp_action::Client<NavigationAction>::SharedPtr nav2_client;
    GoalHandle::SharedPtr goal_handle = nullptr;
    bool finished_navigating = true;

    bool initialize(
      const std::string& ns,
      const std::string& name,
      const std::string& charger_name,
      const std::string& initial_map_name,
      rclcpp::Node::SharedPtr node);

    RobotState get_state();

    GoalCompletedCallback navigate(
      const std::string& map_name,
      const Eigen::Vector3d goal,
      RobotUpdateHandlePtr robot_handle);

    bool stop();

    GoalCompletedCallback dock(
      const std::string& dock_name,
      RobotUpdateHandlePtr robot_handle);

    void action_executor(
      const std::string& category,
      const nlohmann::json& description,
      RobotUpdateHandle::ActionExecution execution);

  };

  struct Data
  {
    std::string fleet_name;
    std::vector<std::thread> threads;
    std::shared_ptr<Configuration> config;
    std::shared_ptr<EasyFullControl> adapter;
    std::unordered_map<std::string, std::shared_ptr<Robot>> robots;
    rclcpp::Node::SharedPtr node;
  };

  void add_robots();
  std::shared_ptr<Data> _data;
};

#endif // SRC__FLEETADAPTER_HPP
