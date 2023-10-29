#ifndef COMMON_H
#define COMMON_H

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>



#include <edge_follower/type.h>
// #include <edge_follower/vizer.h>

namespace edge_follower_ns {

float get_diff_angle(const float angle_start, const float angle_end);

bool in_polygon(const geometry_msgs::msg::Polygon& polygon, const geometry_msgs::msg::Point p);

void create_circle_polygon(geometry_msgs::msg::PolygonStamped& polygon, const float radius);

void create_ellipse_polygon(geometry_msgs::msg::PolygonStamped& polygon,
  const float a, const float b, const float offset);

void transform_polygon(const ef_point_t& check_pose,
                       const geometry_msgs::msg::PolygonStamped& base_polygon,
                       geometry_msgs::msg::PolygonStamped& check_polygon);

void set_cmdvel(const float linear_x, const float angular_z,
                geometry_msgs::msg::Twist& cmd_vel);

bool updateCheckPosesOnPath(
  const nav_msgs::msg::Path& path, std::vector<ef_point_t>& poses,
  const float step, const float len, const int start_idx = 0);

bool isCollisionOnPoses(std::vector<nav2_costmap_2d::MapLocation>& check_cells,
  const std::vector<ef_point_t>& poses,
  const geometry_msgs::msg::PolygonStamped& polygon,
  nav2_costmap_2d::Costmap2D* map, const float sparse_size = -1.0);

bool isCollisionInPolygon(std::vector<nav2_costmap_2d::MapLocation>& check_cells,
    const geometry_msgs::msg::PolygonStamped& check_polygon,
    nav2_costmap_2d::Costmap2D* map, const float sparse_size = -1.0,
    const unsigned char collision_value = nav2_costmap_2d::LETHAL_OBSTACLE);

float findClostest(const nav_msgs::msg::Path& path, 
    const geometry_msgs::msg::PoseStamped& pose, int& index_min);


} // namespace edge_follower_ns

#endif
