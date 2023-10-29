#include <algorithm>
#include <string>
#include <memory>

#include "edge_follower/edge_follower_ros.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include <boost/thread.hpp>

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using namespace nav2_costmap_2d;

namespace edge_follower_ns
{

void EdgeFollowerROS::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  costmap_ = costmap_ros_->getCostmap();
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  
  costmap_ros_ptr_ = costmap_ros;
  costmap_ptr_ = costmap_ros_ptr_->getCostmap();
  // static_costmap_ptr_ = costmap_ros_ptr_->getStaticCostmap() ;

  sp_cfg_ = std::make_shared<edge_follower_ns::Config>(node, plugin_name_);
  sp_cfg_->LoadConfig(params_);

  path_watcher_ = std::make_shared<PathWatcher>(node, &params_);
  edge_follower_ = std::make_shared<EdgeFollower>(node, &params_);
  carrot_track_ = std::make_shared<CarrotTrack>(node, &params_);

  opt_map_ptr_ = std::make_shared<nav2_costmap_2d::Costmap2D>(*costmap_ptr_);
  opt_map_pub_ = std::make_shared<nav2_costmap_2d::Costmap2DPublisher>(
      node, opt_map_ptr_.get(), "map", "opt_map", true);

  sp_thread_ = std::make_shared<std::thread>(
      std::mem_fun(&EdgeFollowerROS::FindDirThread), this);
  ThreadSuspend();

  ref_collision_cells_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud>("ref_collision_cells", 1);
  ref_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("ref_plan", 1);

  initialized_ = true;
  RCLCPP_INFO(logger_, "[EF] init finish !");
}

void EdgeFollowerROS::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type"
    " regulated_pure_pursuit_controller::EdgeFollowerROS",
    plugin_name_.c_str());

  ref_path_pub_.reset();
  ref_collision_cells_pub_.reset();

  path_watcher_.reset();
  edge_follower_.reset();
  carrot_track_.reset();
}

void EdgeFollowerROS::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type "
    "regulated_pure_pursuit_controller::EdgeFollowerROS",
    plugin_name_.c_str());

  ref_path_pub_->on_activate();
  ref_collision_cells_pub_->on_activate();
}

void EdgeFollowerROS::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type "
    "regulated_pure_pursuit_controller::EdgeFollowerROS",
    plugin_name_.c_str());

  ref_path_pub_->on_deactivate();
  ref_collision_cells_pub_->on_deactivate();

  ThreadSuspend();
  boundary_map_.reset();
}

void EdgeFollowerROS::setBoundary(const nav_msgs::msg::Path & boundary)
{
  nav2_costmap_2d::Costmap2D* static_map =
    costmap_ros_ptr_->getStaticCostmap();
  initBoundary(*static_map, boundary);

  UpdateMode();
  create_circle_polygon(ref_circle_polygon_, params_.counter_close2-0.1);
  create_ellipse_polygon(ref_ellipse_polygon_,
    params_.ref_path_check_ellipse_a, params_.ref_path_check_ellipse_b,
    params_.ref_path_check_offset);
}

void EdgeFollowerROS::setPlan(const nav_msgs::msg::Path & path)
{
  path_watcher_->reset();  
  ref_path_ = path;
  InitRefPathKDT();
  ThreadResume();
  SetExcuteTime();
}

geometry_msgs::msg::TwistStamped EdgeFollowerROS::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed)
{
  geometry_msgs::msg::PoseStamped p = pose;
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.twist = speed;
  return cmd_vel;
}

// -------------------- private methods --------------------
void EdgeFollowerROS::FindDirThread()
{
  rclcpp::Rate r(params_.map_update_hz);
  while (rclcpp::ok()) {
    {
      std::unique_lock<std::mutex> lock(th_mutex_);
      while (suspend_) {
        th_cond_.wait(lock);
      }
    }
    if (!opt_map_ok_) {
      r.sleep();
      continue;
    }
    rclcpp::Time st = rclcpp::Time();
    #if 0
    costmap_2d::Costmap2D map;
    if (!CopyMap(*static_costmap_ptr_, map)) {
      ROS_WARN("[EF ROS][%s,%d] copy static map failed !", __FUNCTION__, __LINE__);
      continue;
    }
    addCostmap(map, *boundary_map_);
    path_watcher_ptr_->updateRobotPose(robot_pose_, robot_);
    road_condition_ = path_watcher_ptr_->watching(map);
    #else
    std::shared_ptr<nav2_costmap_2d::Costmap2D> map;
    {
      std::lock_guard<std::mutex> guard(opt_map_lock_);
      map = std::make_shared<nav2_costmap_2d::Costmap2D>(*opt_map_ptr_);
    }
    path_watcher_->updateRobotPose(robot_pose_, robot_);
    road_condition_ = path_watcher_->watching(*map);
    #endif
    // RCLCPP_INFO(logger_, "[EF] find dir speed time: %f !", rclcpp::Time() - st);
    r.sleep();
  }
  sp_thread_->join();
}

bool EdgeFollowerROS::UpdateOptMap()
{
  std::lock_guard<std::mutex> guard(opt_map_lock_);
  if (!CopyMap(*costmap_ptr_, *opt_map_ptr_)) {
    RCLCPP_WARN(logger_, "[EF] copy map failed !");
    return false;
  }
  addCostmap(*opt_map_ptr_, *boundary_map_);
  opt_map_pub_->publishCostmap();
  opt_map_ok_ = true;
  return true;
}

bool EdgeFollowerROS::CopyMap(
  nav2_costmap_2d::Costmap2D& map, nav2_costmap_2d::Costmap2D& copy)
{
  auto map_mutex = map.getMutex();
  boost::recursive_mutex::scoped_lock lock(*map_mutex);
  if (params_.opt_map_size > 0) {
    double ox = map.getOriginX(), oy = map.getOriginY();
    double d = std::hypot(robot_.x - ox, robot_.y - oy);
    double s = params_.opt_map_size / 2;
    if (d > std::hypot(s,s)) {
      ox = robot_.x - s; oy = robot_.y - s;
      if (!copy.copyCostmapWindow(map, ox, oy, params_.opt_map_size, params_.opt_map_size)) {
        RCLCPP_ERROR(logger_, "[EF] ef map copy error !");
        return false;
      }
    } else {
      copy = map;
    }    
  } else {
    copy = map;
  }
  return true;
}

bool EdgeFollowerROS::initBoundary(
  nav2_costmap_2d::Costmap2D& map, const nav_msgs::msg::Path& boundary)
{
  if (boundary.poses.empty()) {
    RCLCPP_ERROR(logger_, "[EF] boundary empty !");
    return false;
  }

  unsigned int width = map.getSizeInCellsX();
  unsigned int height = map.getSizeInCellsY();
  double resolution = map.getResolution();
  double origin_x = map.getOriginX();
  double origin_y = map.getOriginY();
  RCLCPP_INFO(logger_, "[EF] map width:%d, height:%d, resolution:%f, origin_x:%f, origin_y:%f",
    width, height, resolution, origin_x, origin_y);
  
  if (boundary_map_.use_count() == 0) {
    RCLCPP_INFO(logger_, "reset boundary map.");
    boundary_map_.reset();
  }
  boundary_map_ = std::make_shared<nav2_costmap_2d::Costmap2D>(
    width, height, resolution, origin_x, origin_y);

  std::vector<nav2_costmap_2d::MapLocation> boundaryMPs;
  for (auto p : boundary.poses) {
    nav2_costmap_2d::MapLocation mp;
    if (!boundary_map_->worldToMap(p.pose.position.x, p.pose.position.y, mp.x, mp.y)) {
      RCLCPP_ERROR(logger_, "[%s,%d] wp [%f,%f] lies outside map bounds !",
        p.pose.position.x, p.pose.position.y);
      return false;
    }
    boundaryMPs.push_back(mp);
  }

  std::vector<nav2_costmap_2d::MapLocation> cells;
  boundary_map_->polygonOutlineCells(boundaryMPs, cells);
  for (auto c : cells) {
    boundary_map_->setCost(c.x, c.y, nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  RCLCPP_INFO(logger_, "boundary map init finish !!!");
  return true;
}

bool EdgeFollowerROS::addCostmap(nav2_costmap_2d::Costmap2D& src_map,
  const nav2_costmap_2d::Costmap2D& map,
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

void EdgeFollowerROS::UpdateMode()
{
  if (mode_ == "left_wall_follow") {
    params_.mode = kLeft;
    sp_cfg_->UpdatePolygon(kLeft, params_);
  } else if (mode_ == "right_wall_follow") {
    params_.mode = kRight;
    sp_cfg_->UpdatePolygon(kRight, params_);
  }
}

bool EdgeFollowerROS::UpdateRobotPose()
{
  rclcpp::Rate r(100);
  for (int i = 0; i < 3; i ++) {
    if (costmap_ros_ptr_->getRobotPose(robot_pose_)) {
      robot_.x = robot_pose_.pose.position.x;
      robot_.y = robot_pose_.pose.position.y;
      robot_.theta = tf2::getYaw(robot_pose_.pose.orientation);
      robot_.sin_theta = sin(robot_.theta);
      robot_.cos_theta = cos(robot_.theta);
      return true;
    }
    r.sleep();
  }
  return false;
}

bool EdgeFollowerROS::UpdateHz()
{
  double cur_time = rclcpp::Time();
  bool update = false;
  if (last_time_ != 0) {
      float dt = static_cast<float>(cur_time - last_time_);
      hz_ = 1 / dt;
      update = true;
  }

  last_time_ = cur_time;
  return update;
}

bool EdgeFollowerROS::IsHzOk()
{
  if (UpdateHz()) {
    if ((hz_ >= params_.limit_hz - 1)) {
      return true;
    } else {
      RCLCPP_ERROR(logger_, "[EF] hz too low, cur_hz:%f, limit_hz:%f",
        hz_, params_.limit_hz);
    }
  }

  return false;
}

bool EdgeFollowerROS::InitRefPathKDT()
{
  if (ref_path_.poses.empty()) {
    RCLCPP_ERROR(logger_, "[EF] ref path empty !");
    return false;
  }
  ref_path_kdt_.Reset();
  ref_path_kdt_.Init(ref_path_);
  return true;
}

bool EdgeFollowerROS::IsBlockedOnRefPath(const nav_msgs::msg::Path& path, const unsigned int idx)
{
  std::vector<ef_point_t> check_poses; 

  if (!updateCheckPosesOnPath(path, check_poses,
    params_.ref_path_check_step, params_.ref_path_check_len, idx)) {
    return false;
  }

  std::vector<nav2_costmap_2d::MapLocation> check_cells; 
  nav2_costmap_2d::Costmap2D* map = opt_map_ptr_.get();
  if (isCollisionOnPoses(check_cells, check_poses ,ref_ellipse_polygon_, map, 0.1)) {
    // sp_vizer_->VizCheckCells(ref_collision_cells_pub_ , check_cells, map); 
    RCLCPP_WARN(logger_, "[EF] ref circle collision on ref path !");
    return true;
  }
  // sp_vizer_->VizCheckCells(ref_collision_cells_pub_ , check_cells, map); 

  return false;
}

bool EdgeFollowerROS::EdgeFollow(geometry_msgs::msg::Twist& cmd_vel, const ef_dir_t dir)
{
  nav_msgs::msg::Path path_ahead = path_watcher_->getPathAhead();
  nanoflann_port_ns::KDTIndex kdi = ref_path_kdt_.FindClosestPoint(robot_.x, robot_.y, 0.f);
  if (IsBlockedOnRefPath(ref_path_, kdi.idx)) {
    set_cmdvel(0, 0, cmd_vel);
    RCLCPP_WARN(logger_, "[EF] blocked on ref path !");
    return false;
  }
  
  path_watcher_->updateRunTime();

  auto road_condition = road_condition_;
  // carrot track
  if (road_condition.track) {
    nav_msgs::msg::Path path_ahead = path_watcher_->getPathAhead();
    carrot_track_->updateRobotPose(robot_pose_, robot_);
    if (!carrot_track_->tracking(opt_map_ptr_.get(), path_ahead, road_condition, cmd_vel)) {
      RCLCPP_WARN(logger_, "[EF] can't tracking !");
      return false;
    } else {
      leave_ = true;
      return true;
    }
  }

  // edge follow
  rclcpp::Time st = rclcpp::Time();
  edge_follower_->updateRobotPose(robot_pose_, robot_);
  edge_follower_->useLeavePolygon(IsLeave());
  edge_follower_->findLighthouse(ref_path_, kdi.idx, 4.f);
  if (!edge_follower_->following(opt_map_ptr_.get(), road_condition, cmd_vel)) {
    RCLCPP_WARN(logger_, "[EF] can't find steering !");
    RCLCPP_INFO(logger_, "[EF] following speed time: %f !", rclcpp::Time() - st);
    return false;
  }
  RCLCPP_INFO(logger_, "[EF] following speed time: %f !", rclcpp::Time() - st);
  return true;
}

bool EdgeFollowerROS::IsLeave()
{
  if (leave_) {
    std::vector<nav2_costmap_2d::MapLocation> check_cells; 
    geometry_msgs::msg::PolygonStamped polygon, check_polygon;
    polygon.header.frame_id = "base_link";
    polygon.polygon = params_.polygon_avoidance;
    transform_polygon(robot_, polygon, check_polygon);
    bool is_collision = isCollisionInPolygon(check_cells, check_polygon, opt_map_ptr_.get(), 0.1);
    if (is_collision) {
      RCLCPP_INFO(logger_, "[EF] check robot poses collision, leave !");
    } else {
      RCLCPP_INFO(logger_, "[EF] already leave.");
      leave_ = false;
    }
  }

  return leave_;
}

void EdgeFollowerROS::SetExcuteTime()
{
  excute_time_ = rclcpp::Time();
}

bool EdgeFollowerROS::IsExcuteFailed()
{
  rclcpp::Duration timeout(params_.timeout);
  if (rclcpp::Time() - excute_time_ >= timeout) {
    RCLCPP_ERROR(logger_, "[EF] timeout:%f s !", params_.timeout);
    return true;
  }
  return false;
}


}  // namespace edge_follower_ns

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  edge_follower_ns::EdgeFollowerROS,
  nav2_core::ControllerPlus)
