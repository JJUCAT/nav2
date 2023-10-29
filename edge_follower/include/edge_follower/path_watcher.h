#ifndef WATCH_H
#define WATCH_H


#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav2_util/geometry_utils.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <edge_follower/type.h>
#include <edge_follower/common.h>
#include <edge_follower/nanoflann_port.h>
#include <edge_follower/nanoflann_port.h>
namespace edge_follower_ns {


class PathWatcher
{
 public:
  PathWatcher(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, const ef_params_t* params);

  ~PathWatcher() {}

  ef_road_condition_t watching(const nav2_costmap_2d::Costmap2D& map);

  void updateRobotPose(const geometry_msgs::msg::PoseStamped pose, const ef_point_t efp) {
    robot_pose_ = pose;
    robot_ = efp;
  }
  
  nav_msgs::msg::Path getPathAhead() { 
    nav_msgs::msg::Path path;
    {
      std::lock_guard<std::mutex> guard(path_ahead_lock_);
      path = path_ahead_;
    }
    return path;
  }

  bool isEnableContoursDist() {
    if (rclcpp::Time() - run_time_ < rclcpp::Duration(1.f))
      enable_shrink_contours_ = true;
    else enable_shrink_contours_ = false;
    return enable_shrink_contours_;
  }

  void updateRunTime() {
    run_time_ = rclcpp::Time();
  }

  void reset() {
    counter_dist_ = params_->counter_faraway;
    enable_shrink_contours_ = false;
    road_condition_.dir = kDirN;
    road_condition_.track = false;
  }

 private:

  cv::Mat costmap2Image(
    const nav2_costmap_2d::Costmap2D& map, const unsigned char threshold);

  void getCounters(const double radius, const double map_res,
    cv::Mat& mat, std::vector<std::vector<cv::Point>>& counters);

  void Counters2Path(const nav2_costmap_2d::Costmap2D& map,
    const std::vector<std::vector<cv::Point>>& counters,
    std::vector<nav_msgs::msg::Path>& paths);

  bool FindPathAhead(const std::vector<nav_msgs::msg::Path>& contours,
    nav_msgs::msg::Path& path_ahead);

  bool FindEdgePath(const std::vector<nav_msgs::msg::Path>& contours,
    nav_msgs::msg::Path& edge_path);

  bool SortContour(nav_msgs::msg::Path& contour, int& idx);

  size_t CutPath(const nav_msgs::msg::Path& path_ahead,
      nav_msgs::msg::Path& cut_path, const float dist, const int idx);

  void UpdateOrientation(nav_msgs::msg::Path& path);
  ef_road_condition_t GetDir(const nav_msgs::msg::Path& path_ahead, const ef_dir_t last_dir);

  bool IsAStraightLine(const std::vector<cv::Point2f>& points, const float err);
  bool IsAPerpendicularLine(const std::vector<cv::Point2f>& points, const float err);
  bool IsPathAheadStraight(const nav_msgs::msg::Path& path, const float step, const float length);

  void updateCounterDist(const bool straight, const float step);

  bool ready2track();

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_ahead_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_ahead_tail_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_in_horizon_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr path_ahead_dir_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr contours_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>::SharedPtr clocest_pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PolygonStamped>::SharedPtr horizon_polygon_pub_;

  rclcpp::Logger logger_{rclcpp::get_logger("EdgeFollower")};
  geometry_msgs::msg::PoseStamped robot_pose_;
  ef_point_t robot_;  
  nanoflann_port_ns::NanoflannPort kdt_;
  const ef_params_t* params_;
  ef_road_condition_t road_condition_;
  ef_dir_t dir_;
  nav_msgs::msg::Path path_ahead_;
  std::mutex path_ahead_lock_;
  bool straight_ = false;
  float counter_dist_;
  bool enable_shrink_contours_ = false;
  rclcpp::Time run_time_;
};

} // namespace edge_follower_ns

#endif // WATCH_H

