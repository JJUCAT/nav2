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

#ifndef EDGE_PLANNER__EDGE_PLANNER_HPP_
#define EDGE_PLANNER__EDGE_PLANNER_HPP_

#include <memory>
#include <vector>
#include <string>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_core/regional_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"

#include "map_editor.h"

namespace edge_planner_ns
{

class EdgePlanner : public nav2_core::RegionalPlanner
{
public:
  /**
   * @brief constructor
   */
  EdgePlanner();

  /**
   * @brief destructor
   */
  ~EdgePlanner();

  /**
   * @brief Configuring plugin
   * @param parent Lifecycle node pointer
   * @param name Name of plugin map
   * @param tf Shared ptr of TF2 buffer
   * @param costmap_ros Costmap2DROS object
   */
  void configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup lifecycle node
   */
  void cleanup() override;

  /**
   * @brief Activate lifecycle node
   */
  void activate() override;

  /**
   * @brief Deactivate lifecycle node
   */
  void deactivate() override;

  /**
   * @brief Creating a edge path of the region
   * @param boundary boundary of the plan
   * @return std::vector<nav_msgs::msg::Path> of the generated paths
   */
  std::vector<nav_msgs::msg::Path> createPlan(
    const nav_msgs::msg::Path & boundary) override;

protected:
  nav2_costmap_2d::Costmap2D * _costmap;
  rclcpp::Clock::SharedPtr _clock;
  rclcpp::Logger _logger{rclcpp::get_logger("EdgePlanner")};
  std::string _global_frame, _name;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr _raw_plan_publisher;

private:
  bool ScreenShot(const nav_msgs::msg::Path& edge,
                  const nav2_costmap_2d::Costmap2D& map_src,
                  nav2_costmap_2d::Costmap2D& map_window);

  bool FillSuburb(const nav_msgs::msg::Path& edge,
                  nav2_costmap_2d::Costmap2D& map_src,
                  std::shared_ptr<nav2_costmap_2d::Costmap2D>& map);

  void GetFullMapEdge(const nav2_costmap_2d::Costmap2D& map, nav_msgs::msg::Path& edge);

  void Counter2Path(const std::vector<std::vector<cv::Point>>& counters,
    const nav2_costmap_2d::Costmap2D& map, std::vector<nav_msgs::msg::Path>& wall_path_list);
  
  std::shared_ptr<MapEditor> map_editor_;
  float close_inflation_;
  float path_inflation_;
  float smooth_inflation_;
  bool debug_;
};

}  // namespace edge_planner_ns

#endif  // EDGE_PLANNER__EDGE_PLANNER_HPP_
