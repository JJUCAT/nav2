#ifndef PATH_WATCHER_H_
#define PATH_WATCHER_H_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <edge_follow_controller/nanoflann_port.h>
#include <edge_follow_controller/type.h>

namespace edge_follow_controller_ns {

class PathWatcher
{
 public:
  PathWatcher(rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    ef_params_t params) : node_(node), params_(params) {
    logger_ = node_->get_logger();
  }
  ~PathWatcher() {}
  
  bool initBoundary(
    const nav2_costmap_2d::Costmap2D& map, const nav_msgs::msg::Path& boundary);

  void watching(const nav2_costmap_2d::Costmap2D& map);


 private:
  bool addCostmap(
    nav2_costmap_2d::Costmap2D& src_map, const nav2_costmap_2d::Costmap2D& map,
    const unsigned char low_cost = nav2_costmap_2d::LETHAL_OBSTACLE,
    const unsigned char high_cost = nav2_costmap_2d::LETHAL_OBSTACLE);

  cv::Mat costmap2Image(
    const nav2_costmap_2d::Costmap2D& map, const unsigned char threshold);

  void getCounters(const double radius, const double map_res,
    cv::Mat& mat, std::vector<std::vector<cv::Point>>& counters);

  void Counters2Path(const nav2_costmap_2d::Costmap2D& map,
    const std::vector<std::vector<cv::Point>>& counters,
    std::vector<nav_msgs::msg::Path>& paths);

  bool FindPathAhead(const std::vector<nav_msgs::msg::Path>& contours,
    nav_msgs::msg::Path& path_ahead);

  bool SortContour(nav_msgs::msg::Path& contour, int& idx);

  size_t CutPath(const nav_msgs::msg::Path& path_ahead,
      nav_msgs::msg::Path& cut_path, const float dist, const int idx);

  ef_dir_t GetDir(const nav_msgs::msg::Path& path_ahead);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("EdgePlanner")};

  std::shared_ptr<nav2_costmap_2d::Costmap2D> boundary_map_;
  nanoflann_port_ns::NanoflannPort kdt_;
  ef_params_t params_;
  ef_dir_t dir_;
};


} // namespace edge_follow_controller_ns


#endif // PATH_WATCHER_H_
