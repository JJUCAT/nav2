#ifndef EDGE_FOLLOW_CONTROLLER__EDGE_FOLLOW_CONTROLLER_ROS_HPP_
#define EDGE_FOLLOW_CONTROLLER__EDGE_FOLLOW_CONTROLLER_ROS_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>

#include "nav2_core/controller.hpp"
#include "path_watcher.h"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include "edge_follow_controller/type.h"
#include "edge_follow_controller/config.hpp"
#include "edge_follow_controller/path_watcher.h"

namespace edge_follow_controller_ns
{

/**
 * @class edge_follow_controller_ns::EdgeFollowControllerROS
 * @brief Regulated pure pursuit controller plugin
 */
class EdgeFollowControllerROS : public nav2_core::Controller
{
public:
  /**
   * @brief Constructor for edge_follow_controller_ns::EdgeFollowControllerROS
   */
  EdgeFollowControllerROS() = default;

  /**
   * @brief Destrructor for edge_follow_controller_ns::EdgeFollowControllerROS
   */
  ~EdgeFollowControllerROS() override = default;

  /**
   * @brief Configure controller state machine
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros) override;

  /**
   * @brief Cleanup controller state machine
   */
  void cleanup() override;

  /**
   * @brief Activate controller state machine
   */
  void activate() override;

  /**
   * @brief Deactivate controller state machine
   */
  void deactivate() override;

  /**
   * @brief Compute the best command given the current pose and velocity, with possible debug information
   *
   * Same as above computeVelocityCommands, but with debug results.
   * If the results pointer is not null, additional information about the twists
   * evaluated will be in results after the call.
   *
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param results   Output param, if not NULL, will be filled in with full evaluation results
   * @return          Best command
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity) override;

  /**
   * @brief nav2_core setPlan - Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

protected:

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  rclcpp::Logger logger_ {rclcpp::get_logger("EdgeFollowControllerROS")};

  ef_params_t params_;
  
  nav_msgs::msg::Path global_plan_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_path_pub_;
  
  std::shared_ptr<PathWatcher> path_watcher_;


};

}  // namespace edge_follow_controller_ns

#endif  // EDGE_FOLLOW_CONTROLLER__EDGE_FOLLOW_CONTROLLER_ROS_HPP_
