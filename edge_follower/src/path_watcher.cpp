#include <edge_follower/path_watcher.h>
#include <limits>

#include <edge_follower/bspine_port.h>

namespace edge_follower_ns
{

PathWatcher::PathWatcher(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
  const ef_params_t* params)
  : params_(params)
{
  logger_ = node->get_logger();

  path_ahead_pub_ =
    node->create_publisher<nav_msgs::msg::Path>("path_ahead", 1);
  path_ahead_tail_pub_ =
    node->create_publisher<nav_msgs::msg::Path>("path_ahead_tail", 1);
  path_in_horizon_pub_ =
    node->create_publisher<nav_msgs::msg::Path>("path_in_horizon", 1);
  path_ahead_dir_pub_ =
    node->create_publisher<geometry_msgs::msg::PoseStamped>("path_ahead_dir", 1);
  contours_pub_ =
    node->create_publisher<nav_msgs::msg::Path>("contours", 1);
  clocest_pose_pub_ =
    node->create_publisher<geometry_msgs::msg::PoseArray>("ef_closest_poses", 1);
  horizon_polygon_pub_ =
    node->create_publisher<geometry_msgs::msg::PolygonStamped>("ef_horizon_polygon", 1);

  counter_dist_ = params_->counter_faraway;
  road_condition_.dir = kDirN;
  road_condition_.track = false;
}

ef_road_condition_t PathWatcher::watching(const nav2_costmap_2d::Costmap2D& map)
{
  cv::Mat local_mat = costmap2Image(map, 254u);
  std::vector<std::vector<cv::Point>> counters;
  getCounters(counter_dist_, map.getResolution(), local_mat, counters);
  std::vector<nav_msgs::msg::Path> paths;
  Counters2Path(map, counters, paths);

  nav_msgs::msg::Path path_ahead;
  if (FindPathAhead(paths, path_ahead)) {
    {
      std::lock_guard<std::mutex> guard(path_ahead_lock_);
      path_ahead_ = path_ahead;
    }
    // sp_vizer_->VizPath(path_ahead_pub_, path_ahead);
    road_condition_ = GetDir(path_ahead, road_condition_.dir);
    // sp_vizer_->VizDir(path_ahead_dir_pub_, robot_, road_condition_.dir);
    bool straight = IsPathAheadStraight(path_ahead, 0.2, params_->straight_line_check_len);
    updateCounterDist(straight, params_->counter_step);
    road_condition_.straight = straight;
    road_condition_.track = ready2track();
  } else {
    RCLCPP_WARN(logger_, "[EF] lose edge !");
    road_condition_.dir = kDirN;
    road_condition_.track = false;
    road_condition_.straight = false;
  }
  return road_condition_;
}

// -------------------- private methods --------------------

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
      else c = 0;
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
  if (FindEdgePath(contours, tmp_path)) {
    kdt_.Reset();
    kdt_.Init(tmp_path);
    nanoflann_port_ns::KDTIndex kdi = kdt_.FindClosestPoint(robot_.x, robot_.y, 0.f);
    int idx = kdi.idx;
    SortContour(tmp_path, idx);
    CutPath(tmp_path, path_ahead, params_->preview_dist, idx);
    // UpdateOrientation(path_ahead);
    path_ahead.header.frame_id = "map";
    path_ahead.header.stamp = rclcpp::Time();
    bspine_port_ns::BspinePort bspine_port(path_ahead);
    if (!bspine_port.twoOrderSmooth(0.2, 0.05, path_ahead)) {
      RCLCPP_ERROR(logger_, "[EF] smooth failed !");
      return false;
    }
    return true;
  }

  RCLCPP_ERROR(logger_, "[EF] lose edge path !");
  return false;
}

bool PathWatcher::FindEdgePath(const std::vector<nav_msgs::msg::Path>& contours,
  nav_msgs::msg::Path& edge_path)
{
  ef_point_t robot_head = robot_;
  robot_head.x += robot_head.cos_theta * params_->head;
  robot_head.y += robot_head.sin_theta * params_->head;   

  std::vector<std::pair<int, nanoflann_port_ns::KDTIndex>> kdis;
  for (unsigned int i = 0; i < contours.size(); i++) {
    kdt_.Reset();
    kdt_.Init(contours.at(i));
    nanoflann_port_ns::KDTIndex kdi0 = kdt_.FindClosestPoint(robot_.x, robot_.y, 0.f);
    nanoflann_port_ns::KDTIndex kdi1 = kdt_.FindClosestPoint(robot_head.x, robot_head.y, 0.f);
    std::pair<int, nanoflann_port_ns::KDTIndex> key;
    if (kdi0.dist < kdi1.dist) key = std::make_pair(i, kdi0);
    else key = std::make_pair(i, kdi1);
    if (kdis.empty()) kdis.push_back(key);
    else {
      bool insert = false;
      for (unsigned int j = 0; j < kdis.size(); j++) {
        if (key.second.dist < kdis.at(j).second.dist) {
          kdis.insert(kdis.begin() + j, key);
          insert = true;
          break;
        }
      }
      if (!insert) kdis.push_back(key);
    }
  }

  for (unsigned int i = 0; i < kdis.size(); i++) {
    if (kdis.at(i).second.dist < 3.f) {
      edge_path = contours.at(kdis.at(i).first);
      auto p = contours.at(kdis.at(i).first).poses.at(kdis.at(i).second.idx);
      ef_point_t tp; tp.x = p.pose.position.x; tp.y = p.pose.position.y;
      std::vector<ef_point_t> vcp;
      vcp.push_back(tp);
      // sp_vizer_->VizCheckPoses(clocest_pose_pub_, vcp);   
      return true;
    }
  }

  RCLCPP_ERROR(logger_, "[EF] can't find edge path.");
  return false;
}

bool PathWatcher::SortContour(nav_msgs::msg::Path& contour, int& idx)
{
  if (contour.poses.size() <= 1 ) {
    RCLCPP_ERROR(logger_, "[EF] can't sort contour.");
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
  float front_diff = get_diff_angle(front_yaw, robot_.theta);
  float back_diff = get_diff_angle(back_yaw, robot_.theta);

  if (fabs(front_diff) > fabs(back_diff)) {
    reverse(contour.poses.begin(), contour.poses.end());
    // 更新最近点下标
    kdt_.Reset();
    kdt_.Init(contour);
    nanoflann_port_ns::KDTIndex kdi = kdt_.FindClosestPoint(robot_.x, robot_.y, 0.f);
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
  for (unsigned int i = 0; i < tmp_path.poses.size()-1; i++) {
    dist_sum += std::hypot(
      tmp_path.poses.at(i).pose.position.x - tmp_path.poses.at(i+1).pose.position.x,
      tmp_path.poses.at(i).pose.position.y - tmp_path.poses.at(i+1).pose.position.y);
    if (dist_sum <= dist)
      cut_path.poses.push_back(tmp_path.poses.at(i));
  }

  return cut_path.poses.size();
}

void PathWatcher::UpdateOrientation(nav_msgs::msg::Path& path)
{
  if (path.poses.empty()) return;
  if (path.poses.size() == 1) {
    path.poses.front().pose.orientation.w = 1;
    path.poses.front().pose.orientation.x = 0;
    path.poses.front().pose.orientation.y = 0;
    path.poses.front().pose.orientation.z = 0;    
  } else if (path.poses.size() > 1) {
    for (size_t i = 1; i < path.poses.size(); i ++) {
      float yaw = atan2(path.poses.at(i).pose.position.y - path.poses.at(i-1).pose.position.y,
                        path.poses.at(i).pose.position.x - path.poses.at(i-1).pose.position.x);
      auto q = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
      path.poses.at(i).pose.orientation = q;
    }
    path.poses.back().pose.orientation = path.poses.at(path.poses.size()-2).pose.orientation;    
  }
}

ef_road_condition_t PathWatcher::GetDir(const nav_msgs::msg::Path& path_ahead, const ef_dir_t last_dir)
{
  RCLCPP_INFO(logger_, "[EF] dir:%d, mode:%d.", last_dir, params_->mode);
  geometry_msgs::msg::PolygonStamped horizon_polygon, base_polygon;
  if (((params_->mode == kLeft) && (last_dir == kDirR)) ||
      ((params_->mode == kRight) && (last_dir == kDirL))) {
    base_polygon.polygon = params_->polygon_horizon_plus;
  } else {
    base_polygon.polygon = params_->polygon_horizon;
  }

  transform_polygon(robot_, base_polygon, horizon_polygon);
  horizon_polygon.header.frame_id = "map";
  horizon_polygon.header.stamp = rclcpp::Time();
  // sp_vizer_->VizPolygon(horizon_polygon_pub_, horizon_polygon);

  geometry_msgs::msg::Point p;
  nav_msgs::msg::Path path_in_horizon;
  path_in_horizon.header = path_ahead.header;
  for (auto pose = path_ahead.poses.rbegin();
      pose != path_ahead.poses.rend(); pose++) {
    p.x = pose->pose.position.x;
    p.y = pose->pose.position.y;
    if (in_polygon(horizon_polygon.polygon, p)) {
      path_in_horizon.poses.push_back(*pose);
      // break;
    }
  }

  ef_road_condition_t road_condition;
  road_condition.dir = kDirS; 
  if (!path_in_horizon.poses.empty()) {
    p.x = path_in_horizon.poses.front().pose.position.x;
    p.y = path_in_horizon.poses.front().pose.position.y;
    float yaw = atan2(p.y - robot_.y, p.x - robot_.x);
    float diff = get_diff_angle(robot_.theta, yaw);
    float limit = params_->dir_ahead_angle;
    if (diff > limit) road_condition.dir = kDirL;
    else if (diff < -limit) road_condition.dir = kDirR;
    road_condition.lighthouse = path_in_horizon.poses.front();
    RCLCPP_INFO(logger_, "[EF] dir:%d, yaw:%f, diff:%f, limit:%f.",
      road_condition.dir, yaw, diff, limit); 
  }

  // sp_vizer_->VizPath(path_in_horizon_pub_, path_in_horizon);
  return road_condition;
}

bool PathWatcher::IsAStraightLine(const std::vector<cv::Point2f>& points, const float err)
{
  double param = 0; //距离模型中的数值参数C
  double reps = 0.01; //坐标原点与直线之间的距离
  double aeps = 0.01; //角度精度
  cv::Vec4f lines;
  cv::fitLine(points,lines,cv::DIST_L1, param, reps, aeps);
  float k = lines[1]/lines[0];
  float b = lines[3]- k*lines[2];
  float ave_dist = 0, k2 = k * k;
  float dist_min = std::numeric_limits<float>::max();
  float dist_max = std::numeric_limits<float>::lowest();
  for (unsigned int i = 0; i < points.size(); i ++) {
    float x = points.at(i).x;
    float y = points.at(i).y;
    float dist = abs(k * x - y + b)/sqrt(1+k2);
    ave_dist += dist;
    if (dist < dist_min) { dist_min = dist; }
    if (dist > dist_max) { dist_max = dist; }
  }

  int num = points.size();
  ave_dist /= num;
  
  RCLCPP_INFO(logger_, "[EF] k:%f, b:%f, ave_dist:%f.", k, b, ave_dist); 
  return ave_dist < err;
}

bool PathWatcher::IsAPerpendicularLine(const std::vector<cv::Point2f>& points, const float err)
{
  float axis = 0;
  for (unsigned int i = 0; i < points.size(); i ++) {
    axis += points.at(i).x;
  }
  axis /= points.size();

  float ave_dist = 0;
  for (unsigned int i = 0; i < points.size(); i ++) {
    ave_dist += abs(axis - points.at(i).x);
  }
  ave_dist /= points.size();
  
  bool ret = ave_dist < err;
  RCLCPP_INFO(logger_, "[EF] Tstraight:%d, axis:%f, ave_dist:%f.", ret, axis, ave_dist); 
  return ret;
}

bool PathWatcher::IsPathAheadStraight(const nav_msgs::msg::Path& path, const float step,
  const float length)
{
  if (path.poses.size() < 2) return false;

  std::vector<cv::Point2f> points;
  points.push_back(cv::Point2f(path.poses.front().pose.position.x,
                               path.poses.front().pose.position.y));
  float s = 0, ss = 0, d;
  for (unsigned int i = 0; i < path.poses.size()-1; i ++) {
    d = std::hypot(path.poses.at(i).pose.position.y - path.poses.at(i+1).pose.position.y,
                   path.poses.at(i).pose.position.x - path.poses.at(i+1).pose.position.x);
    s += d; ss += d;
    if (s >= step) {
      s = 0;
      points.push_back(cv::Point2f(path.poses.at(i+1).pose.position.x,
                                   path.poses.at(i+1).pose.position.y));
    }
    if (ss >= length) break;
  }

  // 判断是否垂线，再判断斜线
  bool straight = IsAPerpendicularLine(points, 0.07);
  if (!straight) {
    straight = IsAStraightLine(points, params_->straight_line_err);
  }
  
  int index_min = -1;
  float dist_clostest = findClostest(path, robot_pose_, index_min);

  // TODO@LZY：怎么改
  if (straight && dist_clostest < 1.0) {
    float path_yaw = atan2(points.back().y - points.front().y,
                           points.back().x - points.front().x);
    float diff_yaw = get_diff_angle(robot_.theta, path_yaw);
    RCLCPP_INFO(logger_, "[EF] path yaw:%f, robot yaw:%f, diff yaw:%f.",
      path_yaw, robot_.theta, diff_yaw);
    if (diff_yaw < 0.35) {
      RCLCPP_INFO(logger_, "[EF] ready to track path !");
      return true;
    }
  }

  return false;
}

void PathWatcher::updateCounterDist(const bool straight, const float step)
{
  isEnableContoursDist();
  if (straight && enable_shrink_contours_) counter_dist_ -= step;
  else counter_dist_ += params_->counter_dec;
  counter_dist_ = std::min(
    std::max(counter_dist_, params_->counter_close2)
    , params_->counter_faraway);
}

bool PathWatcher::ready2track()
{
  return counter_dist_ < params_->counter_faraway - 0.001;
}

} // namespace edge_follower_ns
