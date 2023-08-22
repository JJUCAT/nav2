// Copyright (c) 2020, Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#include <string>
#include <memory>
#include <vector>
#include <algorithm>
#include <limits>

#include "Eigen/Core"
#include "edge_planner/edge_planner.hpp"

#define BENCHMARK_TESTING

namespace smac_planner
{

using namespace std::chrono;  // NOLINT

EdgePlanner::EdgePlanner()
: _costmap(nullptr)
{
}

EdgePlanner::~EdgePlanner()
{
  RCLCPP_INFO(
    _logger, "Destroying plugin %s of type EdgePlanner",
    _name.c_str());
}

void EdgePlanner::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  std::string name, std::shared_ptr<tf2_ros::Buffer>/*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  _logger = node->get_logger();
  _clock = node->get_clock();
  _costmap = costmap_ros->getCostmap();
  _name = name;
  _global_frame = costmap_ros->getGlobalFrameID();
  _raw_plan_publisher =
    node->create_publisher<nav_msgs::msg::Path>("edge_plan", 1);

  RCLCPP_INFO(
    _logger, "Configured plugin %s of type EdgePlanner.",
    _name.c_str());
}

void EdgePlanner::activate()
{
  RCLCPP_INFO(
    _logger, "Activating plugin %s of type EdgePlanner",
    _name.c_str());
  _raw_plan_publisher->on_activate();
}

void EdgePlanner::deactivate()
{
  RCLCPP_INFO(
    _logger, "Deactivating plugin %s of type EdgePlanner",
    _name.c_str());
  _raw_plan_publisher->on_deactivate();
}

void EdgePlanner::cleanup()
{
  RCLCPP_INFO(
    _logger, "Cleaning up plugin %s of type EdgePlanner",
    _name.c_str());
  _raw_plan_publisher.reset();
}

std::vector<nav_msgs::msg::Path> EdgePlanner::createPlan(
  const nav_msgs::msg::Path & boundary)
{
  std::vector<nav_msgs::msg::Path> plans;
  return plans;
}

}  // namespace smac_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(edge_planner::EdgePlanner, nav2_core::RegionalPlanner)
