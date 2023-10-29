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

namespace edge_planner_ns
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

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".close_inflation", rclcpp::ParameterValue(0.9f));
  node->get_parameter(name + ".close_inflation", close_inflation_);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".path_inflation", rclcpp::ParameterValue(1.2f));
  node->get_parameter(name + ".path_inflation", path_inflation_);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".smooth_inflation", rclcpp::ParameterValue(1.2f));
  node->get_parameter(name + ".smooth_inflation", smooth_inflation_);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".debug", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".debug", debug_);

  RCLCPP_INFO(
    _logger, "Configured plugin %s of type EdgePlanner.",
    _name.c_str());
  RCLCPP_INFO(_logger, "close inflation: %f.", close_inflation_);
  RCLCPP_INFO(_logger, "path inflation: %f.", path_inflation_);
  RCLCPP_INFO(_logger, "smooth inflation: %f.", smooth_inflation_);
  RCLCPP_INFO(_logger, "debug: %f.", debug_);

  map_editor_ = std::make_shared<MapEditor>(&_logger, debug_);
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
  RCLCPP_INFO(_logger, "[EP] boundary size:%u.", boundary.poses.size());

  nav_msgs::msg::Path edge = boundary;
  // GetFullMapEdge(*_costmap, edge); // 测试全图规划，edge 更新为全图的四个顶点
  
  nav2_costmap_2d::Costmap2D map_window;
  if (!ScreenShot(edge, *_costmap, map_window)) {
    RCLCPP_ERROR(_logger, "[EP] copy window failed !");
    return plans;
  }
  
  if (!map_editor_->Costmap2ImageWithBoundary(map_window, edge)) {
    RCLCPP_ERROR(_logger, "[EP] get costmap with boundary failed !");
    return plans;
  }

  std::vector<std::vector<cv::Point>> counters;
  // if (scene == "outer") {
    map_editor_->GetOuterCounter(
      close_inflation_, path_inflation_, smooth_inflation_, counters);
    std::reverse(counters.front().begin(), counters.front().end());
  // } else if (scene == "inner") {
  //   map_editor_->GetInnerCounter(close_inflation_, path_inflation_, counters);
  //   for (auto& counter : counters) {
  //     if (!counter.empty())
  //       std::reverse(counter.begin(), counter.end());
  //   }
  // }

  // TODO@LMR 平滑路径，补充切线角度
  Counter2Path(counters, map_window, plans);
  return plans;
}


// ------------------------------------------------------------
bool EdgePlanner::ScreenShot(const nav_msgs::msg::Path& edge,
  const nav2_costmap_2d::Costmap2D& map, nav2_costmap_2d::Costmap2D& map_window) {
  for (size_t i = 0; i < edge.poses.size(); i ++) {
    RCLCPP_INFO(_logger, "[EP] edge pose[%d]:[%f,%f].",
      i, edge.poses.at(i).pose.position.x, edge.poses.at(i).pose.position.y);
  }
  RCLCPP_INFO(_logger, "[EP] map ox:%f, oy:%f, mx:%f, my:%f.",
    map.getOriginX(), map.getOriginY(), map.getSizeInMetersX(), map.getSizeInMetersY());

  double x_min = std::numeric_limits<double>::max(), y_min = x_min;
  double x_max = std::numeric_limits<double>::min(), y_max = x_max;
  for (auto p : edge.poses) {
    if (p.pose.position.x > x_max) x_max = p.pose.position.x;
    if (p.pose.position.x < x_min) x_min = p.pose.position.x;
    if (p.pose.position.y > y_max) y_max = p.pose.position.y;
    if (p.pose.position.y < y_min) y_min = p.pose.position.y;
  }

  double x_win = x_max - x_min, y_win = y_max - y_min;
  RCLCPP_INFO(_logger, "[EP] x min:%f, x max:%f, y min:%f y max:%f x win:%f, y win:%f",
    x_min, x_max, y_min, y_max, x_win, y_win);

  bool ret = map_window.copyCostmapWindowFully(map, x_min, y_min, x_win, y_win);
  if (ret) {
    RCLCPP_INFO(_logger, "[EP] window map, ox:%f, oy:%f, x max:%f, y max:%f, x in meter:%f, y in meter:%f",
      map_window.getOriginX(), map_window.getOriginY(),
      map_window.getOriginX() + map_window.getSizeInMetersX(),
      map_window.getOriginY() + map_window.getSizeInMetersY(),
      map_window.getSizeInMetersX(), map_window.getSizeInMetersY());
  } else {
    RCLCPP_ERROR(_logger, "[EP] copy window failed !");
  }

  return ret;
}

bool EdgePlanner::FillSuburb(const nav_msgs::msg::Path& edge,
  nav2_costmap_2d::Costmap2D& map_src, std::shared_ptr<nav2_costmap_2d::Costmap2D>& map) {
  std::vector<nav2_costmap_2d::MapLocation> zone;
  nav2_costmap_2d::MapLocation mp;
  for (auto pose : edge.poses) {
    if (!map_src.worldToMap(pose.pose.position.x, pose.pose.position.y, mp.x, mp.y)) {
      RCLCPP_ERROR(_logger, "[EP] Polygon point [%f,%f] lies outside map bounds !",
        pose.pose.position.x, pose.pose.position.y);
      return false;
    }
    zone.push_back(mp);
  }

  std::vector<nav2_costmap_2d::MapLocation> check_cells;
  map_src.convexFillCells(zone, check_cells);

  map = std::make_shared<nav2_costmap_2d::Costmap2D>(
    map_src.getSizeInCellsX(), map_src.getSizeInCellsY(), map_src.getResolution(),
    map_src.getOriginX(), map_src.getOriginY(), nav2_costmap_2d::LETHAL_OBSTACLE);
  for (auto cell : check_cells) {
    map->setCost(cell.x, cell.y, map_src.getCost(cell.x, cell.y));
  }

  return true;
}


void EdgePlanner::GetFullMapEdge(const nav2_costmap_2d::Costmap2D& map, nav_msgs::msg::Path& edge)
{
  edge.header.frame_id = "map";
  edge.header.stamp = rclcpp::Clock().now();
  edge.poses.clear();
  double ox = map.getOriginX(), oy = map.getOriginY();
  double mx = map.getSizeInMetersX(), my = map.getSizeInMetersY();
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = ox; pose.pose.position.y = oy; edge.poses.push_back(pose);
  pose.pose.position.x = ox + mx; pose.pose.position.y = oy; edge.poses.push_back(pose);
  pose.pose.position.x = ox + mx; pose.pose.position.y = oy + my; edge.poses.push_back(pose);
  pose.pose.position.x = ox; pose.pose.position.y = oy + my; edge.poses.push_back(pose);
}

void EdgePlanner::Counter2Path(const std::vector<std::vector<cv::Point>>& counters,
  const nav2_costmap_2d::Costmap2D& map, std::vector<nav_msgs::msg::Path>& wall_path_list) {
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp = rclcpp::Clock().now();
  geometry_msgs::msg::PoseStamped pose;
  pose.header = path.header;
  int map_rows = map.getSizeInCellsY();
  for (size_t i = 0; i < counters.size(); i ++) {
    for (size_t j = 0; j < counters.at(i).size(); j ++) {
      int mx = counters.at(i).at(j).x, my = map_rows - counters.at(i).at(j).y - 1;
      map.mapToWorld(mx, my, pose.pose.position.x, pose.pose.position.y);
      path.poses.push_back(pose);
    }
    for (size_t j = 0; j < path.poses.size(); j ++) {
      if (j + 1 < path.poses.size()) {
        float yaw = atan2(
          path.poses.at(j+1).pose.position.y - path.poses.at(j).pose.position.y,
          path.poses.at(j+1).pose.position.x - path.poses.at(j).pose.position.x);
        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, yaw);
        path.poses.at(j).pose.orientation = tf2::toMsg(quaternion);
      } else {
        path.poses.at(j).pose.orientation = path.poses.back().pose.orientation;
      }
    }
    wall_path_list.push_back(path);
    path.poses.clear();
  }
}


}  // namespace edge_planner_ns

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(edge_planner_ns::EdgePlanner, nav2_core::RegionalPlanner)
