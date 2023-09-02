#include "edge_follow_controller/nanoflann_port.h"
#include "edge_follow_controller/path_watcher.h"
#include "nav2_util/geometyr_utils.hpp"
#include <memory>

namespace edge_follow_controller_ns
{

bool PathWatcher::initBoundary(
  const nav2_costmap_2d::Costmap2D& map, const nav_msgs::msg::Path& boundary)
{
  if (boundary.poses.empty()) {
    RCLCPP_ERROR(logger_, "[%s,%d] boundary empty !", __FUNCTION__, __LINE__);
    return false;
  }

  unsigned int width = map.getSizeInCellsX();
  unsigned int height = map.getSizeInCellsY();
  double resolution = map.getResolution();
  double origin_x = map.getOriginX();
  double origin_y = map.getOriginY();
  RCLCPP_INFO(logger_, "[%s,%d] map width:%d, height:%d, resolution:%f, "
    "origin_x:%f, origin_y:%f", __FUNCTION__, __LINE__,
    width, height, resolution, origin_x, origin_y);
  
  if (boundary_map_.use_count() == 0) {
    RCLCPP_INFO(logger_, "[%s,%d] reset boundary map", __FUNCTION__, __LINE__);
    boundary_map_.reset();
  }
  boundary_map_ = std::make_shared<nav2_costmap_2d::Costmap2D>(
    width, height, resolution, origin_x, origin_y);

  std::vector<nav2_costmap_2d::MapLocation> boundaryMPs;
  for (auto p : boundary.poses) {
    nav2_costmap_2d::MapLocation mp;
    if (!boundary_map_->worldToMap(p.pose.position.x, p.pose.position.y, mp.x, mp.y)) {
      RCLCPP_ERROR(logger_, "[%s,%d] wp [%f,%f] lies outside map bounds !",
          __FUNCTION__, __LINE__, p.pose.position.x, p.pose.position.y);
      return false;
    }
    boundaryMPs.push_back(mp);
  }

  std::vector<nav2_costmap_2d::MapLocation> cells;
  boundary_map_->polygonOutlineCells(boundaryMPs, cells);
  for (auto c : cells) {
    boundary_map_->setCost(c.x, c.y, nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  RCLCPP_INFO(logger_, "[%s,%d] boundary map init finish !!!", __FUNCTION__, __LINE__);
  return true;
}

void PathWatcher::watching(const nav2_costmap_2d::Costmap2D& map)
{
  nav2_costmap_2d::Costmap2D local_map(map);
  addCostmap(local_map, *boundary_map_);
  // 发布地图
  cv::Mat local_mat = costmap2Image(local_map, 244u);
  std::vector<std::vector<cv::Point>> counters;
  getCounters(1.4f, local_map.getResolution(), local_mat, counters);
  std::vector<nav_msgs::msg::Path> paths;
  Counters2Path(local_map, counters, paths);

  nav_msgs::Path path_ahead;
  if (FindPathAhead(paths, path_ahead)) {
      // straight_ = IsPathAheadStraight(path_ahead, 8, 0.1);
      // if (straight_)path_ahead_ = path_ahead;
      // nav_msgs::Path path_ahead_tail;
      // path_ahead.header.frame_id = "map";
      // path_ahead.header.stamp = ros::Time::now();
      // sp_vizer_->VizPath(path_ahead_pub_, path_ahead);
      dir_ = GetDir(path_ahead);
      // sp_vizer_->VizDir(path_ahead_dir_pub_, rbt_, dir_);
  } else {
    RCLCPP_WARN(logger_, "[%s,%d] lose edge !", __FUNCTION__, __LINE__);
    dir_ = kDirN;    
    straight_ = false;
  }
}


// -------------------- private methods --------------------
bool PathWatcher::addCostmap(
  nav2_costmap_2d::Costmap2D& src_map, const nav2_costmap_2d::Costmap2D& map,
  const unsigned char low_cost, const unsigned char high_cost)
{
  //TODO：判断是否重叠，只添加重叠部分
  double wx,wy;
  unsigned int mx, my;
  for (unsigned int y = 0; y < src_map.getSizeInCellsY(); y ++) {
    for (unsigned int x = 0; x < src_map.getSizeInCellsX(); x ++) {
      src_map.mapToWorld(x, y, wx, wy);
      if (!map.worldToMap(wx, wy, mx, my)) {
        continue;
      }
      unsigned char cost = map.getCost(mx, my);
      if (cost >= low_cost && cost <= high_cost) {
        src_map.setCost(x, y, cost);
      }
    }
  }

  return true;
}

cv::Mat PathWatcher::costmap2Image(
  const nav2_costmap_2d::Costmap2D& map, const unsigned char threshold)
{
  unsigned int xs = map.getSizeInCellsX();
  unsigned int ys = map.getSizeInCellsY();
  cv::Mat m( xs, ys, CV_8UC1, cv::Scalar(0));
  for (unsigned int x = 0; x < xs; x++) {
    for (unsigned int y = 0; y < ys; y++) {
      unsigned char c = map.getCost(x, y);
      if (c >= threshold) c = 255u;
      m.at<unsigned char>(y, x) = c;
    }
  }

  return m;
}

void PathWatcher::getCounters(const double radius, const double map_res,
  cv::Mat& mat, std::vector<std::vector<cv::Point>>& counters)
{
  unsigned int r = std::ceil(radius / map_res);
  cv::Mat e = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2*r, 2*r));
  cv::dilate(mat, mat, e);
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(mat, counters, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
}

void PathWatcher::Counters2Path(const nav2_costmap_2d::Costmap2D& map,
  const std::vector<std::vector<cv::Point>>& counters, std::vector<nav_msgs::msg::Path>& paths)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp = rclcpp::Time();
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.header.stamp = rclcpp::Time();

  for (unsigned int i = 0; i < counters.size(); i++) {
    for (unsigned int j = 0; j < counters.at(i).size(); j++) {
      map.mapToWorld(counters.at(i).at(j).x, counters.at(i).at(j).y,
        p.pose.position.x, p.pose.position.y);
      path.poses.push_back(p);
    }
    paths.push_back(path);
    path.poses.clear();
  }
}

bool PathWatcher::FindPathAhead(const std::vector<nav_msgs::msg::Path>& contours,
  nav_msgs::msg::Path& path_ahead)
{
  nav_msgs::msg::Path tmp_path;
  int closest_idx = 0;
  if (FindEdgePath(contours, tmp_path, closest_idx)) {
    kdt_.Reset();
    kdt_.Init(tmp_path);
    nanoflann_port_ns::KDTIndex kdi = kdt_.FindClosestPoint(rbt_.x, rbt_.y, 0.f);
    int idx = kdi.idx;
    SortContour(tmp_path, idx);
    CutPath(tmp_path, path_ahead, params_.preview_dist, idx);
    return true;
  }

  RCLCPP_WARN(logger_, "[%s,%d] lose edge path !", __FUNCTION__, __LINE__);
  return false;
}

bool PathWatcher::SortContour(nav_msgs::msg::Path& contour, int& idx)
{
  if (contour.poses.size() <= 1 ) {
    RCLCPP_ERROR(logger_, "[%s,%d] can't sort contour", __FUNCTION__, __LINE__);
    return false;
  }

  unsigned int closest_idx = idx, front_idx = closest_idx+1, back_idx = closest_idx-1;
  if (closest_idx == contour.poses.size()-1) front_idx = 0;
  if (closest_idx == 0) back_idx = contour.poses.size()-1;
  geometry_msgs::msg::Point closest_pose = contour.poses.at(closest_idx).pose.position;
  geometry_msgs::msg::Point front_pose = contour.poses.at(front_idx).pose.position;
  geometry_msgs::msg::Point back_pose = contour.poses.at(back_idx).pose.position;
  float front_yaw = atan2(front_pose.y - closest_pose.y, front_pose.x - closest_pose.x);
  float back_yaw = atan2(back_pose.y - closest_pose.y, back_pose.x - closest_pose.x);
  float front_diff = GetDiffAngle(front_yaw, rbt_.theta);
  float back_diff = GetDiffAngle(back_yaw, rbt_.theta);

  if (fabs(front_diff) > fabs(back_diff)) {
    reverse(contour.poses.begin(), contour.poses.end());
    // 更新最近点下标
    kdt_.Reset();
    kdt_.Init(contour);
    nanoflann_port_ns::KDTIndex kdi = kdt_.FindClosestPoint(rbt_.x, rbt_.y, 0.f);
    idx = kdi.idx;
  }

  return true;
}

size_t PathWatcher::CutPath(const nav_msgs::msg::Path& path_ahead,
    nav_msgs::msg::Path& cut_path, const float dist, const int idx)
{
  nav_msgs::msg::Path tmp_path = path_ahead;
  tmp_path.poses.assign(path_ahead.poses.begin()+idx, path_ahead.poses.end());
  tmp_path.poses.insert(tmp_path.poses.end(), path_ahead.poses.begin(), path_ahead.poses.begin()+idx);

  float dist_sum = 0;
  cut_path.header = tmp_path.header;
  for (int i = 0; i < tmp_path.poses.size()-1; i++) {
    dist_sum += std::hypot(
      tmp_path.poses.at(i).pose.position.x - tmp_path.poses.at(i+1).pose.position.x,
      tmp_path.poses.at(i).pose.position.y - tmp_path.poses.at(i+1).pose.position.y);
    if (dist_sum <= dist)
      cut_path.poses.push_back(tmp_path.poses.at(i));
  }

  return cut_path.poses.size();
}

ef_dir_t EdgeFollowerROS::GetDir(const nav_msgs::msg::Path& path_ahead)
{
  geometry_msgs::msg::PolygonStamped horizon_polygon, base_polygon;
  if (((params_.mode == kLeft) && (dir_ == kDirR)) || ((params_.mode == kRight) && (dir_ == kDirL))) {
    base_polygon.polygon = params_.polygon_horizon_plus;
  } else {
    base_polygon.polygon = params_.polygon_horizon;
  }

  TransformPolygon(rbt_, base_polygon, horizon_polygon);
  horizon_polygon.header.frame_id = "map";
  horizon_polygon.header.stamp = rclcpp::Time();
  // sp_vizer_->VizPolygon(horizon_polygon_pub_, horizon_polygon);

  geometry_msgs::msg::Point p;
  bool find = false;
  nav_msgs::msg::Path path_in_horizon;
  path_in_horizon.header = path_ahead.header;
  for (auto pose = path_ahead.poses.rbegin();
      pose != path_ahead.poses.rend(); pose++) {
    p.x = pose->pose.position.x;
    p.y = pose->pose.position.y;
    if (nav2_util::geometry_utils::in_polygon(horizon_polygon.polygon, p)) {
        path_in_horizon.poses.push_back(*pose);
        // break;
    }
  }

  ef_dir_t dir = kDirS;
  if (!path_in_horizon.poses.empty()) {
    p.x = path_in_horizon.poses.front().pose.position.x;
    p.y = path_in_horizon.poses.front().pose.position.y;
    float yaw = atan2(p.y - rbt_.y, p.x - rbt_.x);
    float diff = nav2_util::geometry_utils::get_diff_angle(rbt_.theta, yaw);
    float limit = params_.dir_ahead_angle;
    if (diff > limit) dir = kDirL;
    else if (diff < -limit) dir = kDirR;
  }

  // sp_vizer_->VizPath(path_in_horizon_pub_, path_in_horizon);
  return dir;
}


} // namespace edge_follow_controller_ns
