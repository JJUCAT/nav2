#ifndef MAP_EDITOR_H
#define MAP_EDITOR_H

#include <memory>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <cv_bridge/cv_bridge.h>

namespace wall_path_planner_ns
{
class MapEditor {
 public:
  MapEditor(bool debug) : debug_(debug) { mat_ = std::make_shared<cv::Mat>(); }

  ~MapEditor() {}

  void Costmap2Image(const costmap_2d::Costmap2D& map);
  bool GetOuterCounter(float close_inflation, float path_inflation,
    float smooth_inflation, std::vector<std::vector<cv::Point>>& outer_counter);
  bool GetInnerCounter(float close_inflation, float path_inflation,
    std::vector<std::vector<cv::Point>>& inner_counters);
  
 private:
  void Inflat(cv::Mat& mat, const float radius, const float resolution);
  void Erode(cv::Mat& mat, const float radius, const float resolution);
  bool Separation();

  std::shared_ptr<cv::Mat> mat_;
  std::shared_ptr<cv::Mat> coast_mat_;
  std::shared_ptr<cv::Mat> mediterranean_mat_;
  float resolution_;
  bool debug_;
};

} // namespace wall_path_planner_ns
#endif
