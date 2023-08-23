#include "map_editor.h"
#include "costmap_2d/cost_values.h"
#include "costmap_2d/costmap_2d.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/duration.h"
#include "tf2/utils.h"
#include "tf/transform_datatypes.h"
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <string>

namespace wall_path_planner_ns
{

void MapEditor::Costmap2Image(const costmap_2d::Costmap2D& map) {
  ROS_INFO("[WPP][%s,%d]", __FUNCTION__, __LINE__);

  int mx = map.getSizeInCellsX(), my = map.getSizeInCellsY();
  resolution_ = map.getResolution();
  ROS_INFO("[WPP][%s,%d] map x:%d, y:%d, res:%f",
    __FUNCTION__, __LINE__, mx, my, resolution_);
  if (mx == 0 || my == 0) {
    ROS_ERROR("[WPP][%s,%d] map error, x:%d, y:%d", __FUNCTION__, __LINE__, mx, my);
    return;
  }

  *mat_ = cv::Mat::zeros(map.getSizeInCellsY(), map.getSizeInCellsX(), CV_8UC1);
  for(int x = 0; x < map.getSizeInCellsX(); x ++) {
    for(int y = 0; y < map.getSizeInCellsY(); y ++) {
      mat_->at<uchar>(map.getSizeInCellsY() - y - 1, x) = map.getCost(x, y) >= costmap_2d::LETHAL_OBSTACLE ? 255 : 0;
    }
  }

  std::string save_path = getenv("HOME");
  save_path = save_path + "/wp_Costmap2Image.jpg";
  cv::imwrite(save_path, *mat_);
}

bool MapEditor::GetOuterCounter(float close_inflation, float path_inflation,
  float smooth_inflation, std::vector<std::vector<cv::Point>>& outer_counter) {
  ROS_INFO("[WPP][%s,%d]", __FUNCTION__, __LINE__);
  Inflat(*mat_, close_inflation, resolution_);
  Separation();

  if (close_inflation > path_inflation) {
    ROS_WARN("[WPP][%s,%d] close_inflation:%f > path_inflation:%f,use close_inflation as path",
      __FUNCTION__, __LINE__, close_inflation, path_inflation);
  } else {
    float inflation = path_inflation - close_inflation;
    ROS_INFO("[WPP][%s,%d] advance inflation:%f", __FUNCTION__, __LINE__, inflation);
    Inflat(*coast_mat_, inflation, resolution_);
  }

  if (smooth_inflation > 0) {
    unsigned int dr = std::ceil(smooth_inflation / resolution_);
    cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2*dr, 2*dr));
    cv::morphologyEx(*coast_mat_, *coast_mat_, cv::MORPH_CLOSE, element);
  }

  std::vector<std::vector<cv::Point>> cs;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(*coast_mat_, cs, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
  cv::Mat cm = cv::Mat::zeros(coast_mat_->size(), CV_8UC1);
  cv::drawContours(cm, cs, -1, 254,1);

  if (cs.empty()) {
    ROS_ERROR("[WPP][%s,%d] can't find counters !", __FUNCTION__, __LINE__);
    return false;
  }

  if (debug_) {
    std::string save_path = getenv("HOME");
    save_path = save_path + "/outer_counters.jpg";
    cv::imwrite(save_path, cm);    
  }

  int map_boundary_idx = -1;
  ROS_INFO("[WPP][%s,%d] hierarchy count:%lu", __FUNCTION__, __LINE__, hierarchy.size());
  for (int i = 0; i < hierarchy.size(); i ++) {
    ROS_INFO("[WPP][%s,%d] hierarchy [%d], back:%d, front:%d, child:%d, parent:%d",
      __FUNCTION__, __LINE__, i,
      hierarchy.at(i)[0], hierarchy.at(i)[1], hierarchy.at(i)[2], hierarchy.at(i)[3]);
    if (hierarchy.at(i)[3] == -1) {
      map_boundary_idx = i;
    }
  }

  size_t size = 0;
  int idx = -1;
  for (int i = 0; i < cs.size(); i ++) {
    if (i == map_boundary_idx) continue;
    if (cs.at(i).size() > size) {
      size = cs.at(i).size();
      idx = i;
    }
  }
  ROS_ERROR("[WPP][%s,%d] outer counter idx:%d !", __FUNCTION__, __LINE__, idx);
  outer_counter.push_back(cs.at(idx));
  return true;
}

bool MapEditor::GetInnerCounter(float close_inflation, float path_inflation,
  std::vector<std::vector<cv::Point>>& inner_counters) {
  ROS_INFO("[WPP][%s,%d]", __FUNCTION__, __LINE__);
  Separation();
  Inflat(*mediterranean_mat_, path_inflation, resolution_);

  std::vector<std::vector<cv::Point>> cs;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(*mediterranean_mat_, cs, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
  cv::Mat cm = cv::Mat::zeros(mediterranean_mat_->size(), CV_8UC1);
  cv::drawContours(cm, cs, -1, 254,1);

  if (cs.empty()) {
    ROS_ERROR("[WPP][%s,%d] can't find counters !", __FUNCTION__, __LINE__);
    return false;
  }

  if (debug_) {
    std::string save_path = getenv("HOME");
    save_path = save_path + "/inner_counters.jpg";
    cv::imwrite(save_path, cm);    
  }

  inner_counters = cs;
  return true;
}

// ------------------------------------------------------------
void MapEditor::Inflat(cv::Mat& mat, const float radius, const float resolution) {
  ROS_INFO("[WPP][%s,%d]", __FUNCTION__, __LINE__);

  unsigned int dr = std::ceil(radius / resolution);
  cv::Mat de;
  de = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2*dr, 2*dr));
  cv::dilate(mat, mat, de);
  
  // std::string save_path = getenv("HOME");
  // save_path = save_path + "/wp_Inflat.jpg";
  // cv::imwrite(save_path, mat);
}

void MapEditor::Erode(cv::Mat& mat, const float radius, const float resolution) {
  ROS_INFO("[WPP][%s,%d]", __FUNCTION__, __LINE__);

  unsigned int er = std::ceil(radius / resolution);
  cv::Mat ee;
    ee = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2*er, 2*er));
  cv::dilate(mat, mat, ee);

  // std::string save_path = getenv("HOME");
  // save_path = save_path + "/wp_Erode.jpg";
  // cv::imwrite(save_path, mat);
}

bool MapEditor::Separation() {
  ROS_INFO("[WPP][%s,%d]", __FUNCTION__, __LINE__);

  cv::Mat areas;
  coast_mat_.reset();
  mediterranean_mat_.reset();
  coast_mat_ = std::make_shared<cv::Mat>(mat_->rows, mat_->cols, CV_8UC1, cv::Scalar(0));
  mediterranean_mat_ = std::make_shared<cv::Mat>(mat_->rows, mat_->cols, CV_8UC1, cv::Scalar(0));

	for (int row = 0; row < mat_->rows; row ++) {
		for (int col = 0; col < mat_->cols; col ++) {
      if (row == 0 || col == 0 || row == mat_->rows-1 || col == mat_->cols-1)
        mat_->at<uchar>(row, col) = 255;
      else
        continue;
		}
	}

  int number = connectedComponents(*mat_, areas, 8, CV_16U);  
  if (number <= 1) {
    ROS_ERROR("[WPP][%s,%d] separate failed, number:%d", __FUNCTION__, __LINE__, number);
    return false;
  }

  int coast_bel = areas.at<uint16_t>(0, 0);
	for (int row = 0; row < mat_->rows; row ++) {
		for (int col = 0; col < mat_->cols; col ++) {
			int label = areas.at<uint16_t>(row, col);
			if (label == 0) { 
        continue; // 背景色黑色
			} else if (label == coast_bel) {
				coast_mat_->at<uchar>(row, col) = mat_->at<uchar>(row, col);
      } else {
        mediterranean_mat_->at<uchar>(row, col) = mat_->at<uchar>(row, col);
      }
		}
	}

  cv::RNG rng(10010);
  std::vector<cv::Vec3b> colors;
  for (int i=0; i<number; i++) {
    cv::Vec3b vec = cv::Vec3b(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
    colors.push_back(vec);
  }
  cv::Mat color_areas_mat = cv::Mat::zeros(mat_->size(), CV_8UC3);
  for (int i = 0; i < mat_->rows; ++i) {
    for (int j = 0; j < mat_->cols; ++j) {
      int label = areas.at<uint16_t>(i, j);
      if (label == 0) continue;
      color_areas_mat.at<cv::Vec3b>(i, j) = colors[label];
    }
  }

  if (debug_) {
    std::string save_path = getenv("HOME");
    std::string coast_path = save_path + "/wp_Coast.jpg";
    std::string mediterranean_path = save_path + "/wp_Mediterranean.jpg";
    std::string color_areas_path = save_path + "/wp_ColorAreas.jpg";
    cv::imwrite(coast_path, *coast_mat_);
    cv::imwrite(mediterranean_path, *mediterranean_mat_);
    cv::imwrite(color_areas_path, color_areas_mat);
  }

  return true;
}

} // namespace wall_path_planner_ns
