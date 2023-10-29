#ifndef CARROT_TRACK_H
#define CARROT_TRACK_H


#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

#include <edge_follower/type.h>
#include <edge_follower/common.h>
#include <edge_follower/motion.h>
#include <edge_follower/nanoflann_port.h>

namespace edge_follower_ns {


class CarrotTrack
{
 public:
  CarrotTrack(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, const ef_params_t* params);

  ~CarrotTrack() {}
  
  void updateMap(nav2_costmap_2d::Costmap2D* map) { map_ = map; }

  void updateRobotPose(const geometry_msgs::msg::PoseStamped pose, const ef_point_t efp) {
    robot_pose_ = pose;
    robot_ = efp;
  }

  bool tracking(nav2_costmap_2d::Costmap2D* map,
    const nav_msgs::msg::Path path, const ef_road_condition_t& road_condition,
    geometry_msgs::msg::Twist& cmd_vel);

 private:
  bool getCarrot(const nav_msgs::msg::Path& path,
    const geometry_msgs::msg::PoseStamped pose,
    const float in_range, const float len, geometry_msgs::msg::PoseStamped& carrot);

  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr steer_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr anchor_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud>::SharedPtr collision_cells_pub_;

  rclcpp::Logger logger_{rclcpp::get_logger("EdgeFollower")};
  std::shared_ptr<Motion> motion_;
  geometry_msgs::msg::PoseStamped robot_pose_;
  ef_point_t robot_;  
  nav2_costmap_2d::Costmap2D* map_;
  const ef_params_t* params_;
  std::vector<ef_point_t> collision_check_poses_;
  std::vector<nav2_costmap_2d::MapLocation> collision_cells_;
  
  geometry_msgs::msg::PolygonStamped circle_polygon_;

};

} // namespace edge_follower_ns

#endif // CARROT_TRACK_H

