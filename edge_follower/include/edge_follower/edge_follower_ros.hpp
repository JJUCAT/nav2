#ifndef EDGE_FOLLOW_CONTROLLER__EDGE_FOLLOW_CONTROLLER_ROS_HPP_
#define EDGE_FOLLOW_CONTROLLER__EDGE_FOLLOW_CONTROLLER_ROS_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>

#include "edge_follower/carrot_track.h"
#include "nav2_core/controller_plus.hpp"
#include "path_watcher.h"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include "edge_follower/type.h"
#include "edge_follower/config.hpp"
#include "edge_follower/path_watcher.h"
#include "edge_follower/carrot_track.h"
#include "edge_follower/edge_follower.h"

namespace edge_follower_ns
{

/**
 * @class edge_follower_ns::EdgeFollowerROS
 * @brief Regulated pure pursuit controller plugin
 */
class EdgeFollowerROS : public nav2_core::ControllerPlus
{
public:
  /**
   * @brief Constructor for edge_follower_ns::EdgeFollowerROS
   */
  EdgeFollowerROS() = default;

  /**
   * @brief Destrructor for edge_follower_ns::EdgeFollowerROS
   */
  ~EdgeFollowerROS() override = default;

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
   * @brief nav2_core setBoundary - Sets the boundary plan
   * @param path The boundary plan
   */
  void setBoundary(const nav_msgs::msg::Path & boundary) override;

  /**
   * @brief nav2_core setPlan - Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

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

protected:

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  rclcpp::Logger logger_ {rclcpp::get_logger("EdgeFollower")};

private:
  bool isInitialized() { return initialized_; }

  void UpdateMode();

  bool UpdateRobotPose();

  bool UpdateHz();

  bool IsHzOk();

  void FindDirThread();

  bool initBoundary(
    nav2_costmap_2d::Costmap2D& map, const nav_msgs::msg::Path& boundary);

  bool addCostmap(nav2_costmap_2d::Costmap2D& src_map, const nav2_costmap_2d::Costmap2D& map,
    const unsigned char low_cost = nav2_costmap_2d::LETHAL_OBSTACLE,
    const unsigned char high_cost = nav2_costmap_2d::LETHAL_OBSTACLE);

  bool UpdateOptMap();

  bool CopyMap(nav2_costmap_2d::Costmap2D& map, nav2_costmap_2d::Costmap2D& copy);

  bool UpdateCheckPosesOnPath(const nav_msgs::msg::Path& path, const float step, const float len, const int start_idx = 0);
  bool EdgeFollow(geometry_msgs::msg::Twist& cmd_vel);

  void SetExcuteTime();
  bool IsExcuteFailed();
  
  bool InitRefPathKDT();
  bool IsBlockedOnRefPath(const nav_msgs::msg::Path& path, const unsigned int idx);

  bool IsLeave();

  std::shared_ptr<std::thread> sp_thread_;
  std::mutex th_mutex_;
  std::condition_variable th_cond_;
  bool suspend_ = true;
  void RecvStaticMap(const nav_msgs::msg::OccupancyGrid::SharedPtr new_map);
  void ThreadSuspend() {
    std::unique_lock<std::mutex> lock(th_mutex_);
    suspend_ = true;
  }
  void ThreadResume() {
    {
      std::unique_lock<std::mutex> lock(th_mutex_);
      suspend_ = false;        
    }
    th_cond_.notify_one();
  }

  bool initialized_;
  ef_params_t params_;
  nav_msgs::msg::Path ref_path_;
  nanoflann_port_ns::NanoflannPort kdt_;
  nanoflann_port_ns::NanoflannPort ref_path_kdt_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_ptr_;
  nav2_costmap_2d::Costmap2D* costmap_ptr_;
  nav2_costmap_2d::Costmap2D* static_costmap_ptr_;

  std::shared_ptr<nav2_costmap_2d::Costmap2D> boundary_map_;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> opt_map_ptr_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DPublisher> opt_map_pub_;
  std::mutex opt_map_lock_;
  bool opt_map_ok_ = false;

  std::shared_ptr<PathWatcher> path_watcher_;
  std::shared_ptr<CarrotTrack> carrot_track_;
  std::shared_ptr<EdgeFollower> edge_follower_;

  rclcpp::Time excute_time_;
  std::string mode_ = "right_wall_follow";

  ef_point_t robot_;
  geometry_msgs::msg::PoseStamped robot_pose_;
  ef_road_condition_t road_condition_;
  ef_dir_t dir_;
  bool finish_plan_;
  bool leave_ = false;
  double last_time_;
  bool map_received_;
  float hz_;
  std::shared_ptr<edge_follower_ns::Config> sp_cfg_;

  std::vector<nav2_costmap_2d::MapLocation> ref_collision_cells_;
  geometry_msgs::msg::PolygonStamped ref_circle_polygon_;
  geometry_msgs::msg::PolygonStamped ref_ellipse_polygon_;
  geometry_msgs::msg::Twist last_cmd_vel_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud>> ref_collision_cells_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> ref_path_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
};

}  // namespace edge_follower_ns

#endif  // EDGE_FOLLOW_CONTROLLER__EDGE_FOLLOW_CONTROLLER_ROS_HPP_
