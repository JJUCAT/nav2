#include <algorithm>
#include <string>
#include <memory>

#include "edge_follow_controller/edge_follow_controller_ros.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using namespace nav2_costmap_2d;

namespace edge_follow_controller_ns
{

void EdgeFollowControllerROS::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  
  path_watcher_ = std::make_shared<PathWatcher>(node, tf, params_);

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.5));

  // node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);

  global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
}

void EdgeFollowControllerROS::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type"
    " regulated_pure_pursuit_controller::EdgeFollowControllerROS",
    plugin_name_.c_str());
  global_path_pub_.reset();
  path_watcher_.reset();
}

void EdgeFollowControllerROS::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type "
    "regulated_pure_pursuit_controller::EdgeFollowControllerROS",
    plugin_name_.c_str());
  global_path_pub_->on_activate();

}

void EdgeFollowControllerROS::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type "
    "regulated_pure_pursuit_controller::EdgeFollowControllerROS",
    plugin_name_.c_str());
  global_path_pub_->on_deactivate();
}

geometry_msgs::msg::TwistStamped EdgeFollowControllerROS::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed)
{
  geometry_msgs::msg::PoseStamped p = pose;
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.twist = speed;
  return cmd_vel;
}

}  // namespace nav2_pure_pursuit_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  edge_follow_controller_ns::EdgeFollowControllerROS,
  nav2_core::Controller)
