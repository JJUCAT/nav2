#ifndef EDGE_FOLLOWER_H_
#define EDGE_FOLLOWER_H_

#include "nav2_util/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include <edge_follow_controller/nanoflann_port.h>
#include <edge_follow_controller/type.h>

namespace edge_follow_controller_ns {

class EdgeFollower
{
 public:
  EdgeFollower(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    const std::shared_ptr<tf2_ros::Buffer>& tf, ef_params_t params)
      : node_(node), tf_(tf), params_(params) {
    logger_ = node_->get_logger();
  }
  ~EdgeFollower() {}
  
  bool follow(nav2_costmap_2d::Costmap2D& map, const ef_dir_t dir,
    geometry_msgs::msg::Twist& cmd_vel);

 private:
  void UpdateSteeringArea(const ef_dir_t dir);
  void UpdateCheckTime(const ef_dir_t dir);
  bool FollowAhead(geometry_msgs::msg::Twist& cmd_vel);
  bool UpdateRobotPose();

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("EdgeFollowController")};
  std::shared_ptr<tf2_ros::Buffer> tf_;
  geometry_msgs::msg::PoseStamped robot_pose_;
  ef_point_t rbt_;  
  std::shared_ptr<nav2_costmap_2d::Costmap2D> boundary_map_;
  nanoflann_port_ns::NanoflannPort kdt_;
  ef_params_t params_;
  ef_dir_t dir_;

  float check_time_;
  float check_time_step_;
  float steering_left_, steering_right_;
};


} // namespace edge_follow_controller_ns


#endif // EDGE_FOLLOWER_H_
