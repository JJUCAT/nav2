#ifndef MAP_EDITOR_H
#define MAP_EDITOR_H

#include <memory>
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/path.hpp"
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace edge_planner_ns
{
class MapEditor {
 public:
  MapEditor(rclcpp::Logger* logger, bool debug) :
    _logger(logger), debug_(debug) { mat_ = std::make_shared<cv::Mat>(); }

  ~MapEditor() {}
  bool Costmap2ImageWithBoundary(const nav2_costmap_2d::Costmap2D& map, const nav_msgs::msg::Path& boundary);
  void Costmap2Image(const nav2_costmap_2d::Costmap2D& map);
  bool GetOuterCounter(float close_inflation, float path_inflation,
    float smooth_inflation, std::vector<std::vector<cv::Point>>& outer_counter);
  bool GetInnerCounter(float close_inflation, float path_inflation, float area_filter,
    std::vector<std::vector<cv::Point>>& inner_counters);
  
 private:
  void Inflat(cv::Mat& mat, const float radius, const float resolution);
  void Erode(cv::Mat& mat, const float radius, const float resolution);
  bool Separation();
  
  std::string save_dir_ = "/ws/navigation2_ws/roslog/";
  rclcpp::Node* node_;
  rclcpp::Logger* _logger;
  std::shared_ptr<cv::Mat> mat_;
  std::shared_ptr<cv::Mat> coast_mat_;
  std::shared_ptr<cv::Mat> mediterranean_mat_;
  float resolution_;
  bool debug_;
};

} // namespace edge_planner_ns
#endif
