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

//==============================================================================
class FleetAdapter : public rclcpp::Node
{
public:
  using EasyFullControl = rmf_fleet_adapter::agv::EasyFullControl;
  using Graph = EasyFullControl::Graph;
  using VehicleTraits = EasyFullControl::VehicleTraits;
  using Configuration = EasyFullControl::Configuration;
  using GetStateCallback = EasyFullControl::GetStateCallback;
  using GoalCompletedCallback = EasyFullControl::GoalCompletedCallback;
  using NavigationRequest = EasyFullControl::NavigationRequest;
  using StopRequest = EasyFullControl::StopRequest;
  using DockRequest = EasyFullControl::DockRequest;

  using NavigationAction = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigationAction>;
  using Odom = nav_msgs::msg::Odometry;

  /// Constructor
  FleetAdapter(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  ~FleetAdapter();

private:
  struct Robot
  {
    std::string name;

  };

  std::string _fleet_name;
  std::thread _thread;
  std::shared_ptr<Configuration> _config;
  std::shared_ptr<EasyFullControl> _adapter;
  std::unordered_map<std::string, std::shared_ptr<Robot>> _robots;

};


#endif // SRC__FLEETADAPTER_HPP