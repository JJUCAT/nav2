#include <edge_follower/carrot_track.h>
#include <limits>


namespace edge_follower_ns
{

CarrotTrack::CarrotTrack(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
  const ef_params_t* params)
  : params_(params)
{
  logger_ = node->get_logger();
  motion_ = std::make_shared<MotionAckerman>(params_);

  steer_pub_ =
    node->create_publisher<visualization_msgs::msg::Marker>("steering", 1);
  anchor_pub_ =
    node->create_publisher<visualization_msgs::msg::Marker>("target", 1);
  collision_cells_pub_ =
    node->create_publisher<sensor_msgs::msg::PointCloud>("collision_cells", 1);

  create_circle_polygon(circle_polygon_, params_->straight_line_check_radius);
}

bool CarrotTrack::tracking(nav2_costmap_2d::Costmap2D* map,
  const nav_msgs::msg::Path path, const ef_road_condition_t& road_condition,
  geometry_msgs::msg::Twist& cmd_vel)
{
  float radius = params_->counter_close2 - 0.1;
  float len = 2;
  std::vector<ef_point_t> check_poses;  
  if (!updateCheckPosesOnPath(path, check_poses, radius / 2, len)) {
    RCLCPP_ERROR(logger_, "[EF] update check poses failed !");
    return false;
  }
  std::vector<nav2_costmap_2d::MapLocation> check_cells; 
  if (isCollisionOnPoses(check_cells, check_poses, circle_polygon_, map, 0.1)) {
    RCLCPP_ERROR(logger_, "[EF] circle collision on path ahead !");
    return false;
  }
  // sp_vizer_->VizCheckCells(collision_cells_pub_ , check_cells, map); 

  geometry_msgs::msg::PoseStamped steering_pose = robot_pose_;
  steering_pose.pose.position.x += params_->stanley_head * robot_.cos_theta;
  steering_pose.pose.position.y += params_->stanley_head * robot_.sin_theta;
  // sp_vizer_->VizMarker(steer_pub_, steering_pose,
  //   visualization_msgs::Marker::CUBE, 0.3, 0, 1.0, 0, 1.0);

  geometry_msgs::msg::PoseStamped carrot;
  if (!getCarrot(path, steering_pose, 0.2, 0.0, carrot)) {
    RCLCPP_ERROR(logger_, "[EF] can't find carrot !");
    return false;
  }
  // sp_vizer_->VizMarker(anchor_pub_, carrot, visualization_msgs::Marker::SPHERE,
  //   0.3, 1.0, 0, 0, 1.0);

  float steering = motion_->StanleyFollow(steering_pose, carrot,
    params_->stanley_head, params_->stanley_mu, params_->stanley_lambda,
    params_->stanley_ks, cmd_vel.linear.x);
  steering = std::clamp(steering, 
    -params_->steering_angle_max, params_->steering_angle_max);

  float speed = motion_->GetSpeed(cmd_vel.linear.x, steering, road_condition.straight);    
  // float speed = motion_->GetTurnSpeed(steering);
  RCLCPP_INFO(logger_, "[EF], steering:%f, speed:%f", steering, speed);
  set_cmdvel(speed, steering, cmd_vel);
  return true;
}

// -------------------- private methods --------------------
bool CarrotTrack::getCarrot(const nav_msgs::msg::Path& path,
  const geometry_msgs::msg::PoseStamped pose,
  const float in_range, const float len, geometry_msgs::msg::PoseStamped& carrot)
{
  if (path.poses.empty()) {
    RCLCPP_ERROR(logger_, "[EF], path error !");
    return false;
  }
  float dist = 0, dist_min = std::numeric_limits<float>::max();
  int index_min = -1;
  for (int i = 0; i < path.poses.size(); i ++) {
    dist = std::hypot(path.poses.at(i).pose.position.x - pose.pose.position.x,
                      path.poses.at(i).pose.position.y - pose.pose.position.y);
    // if (dist < in_range) {
    //   index_min = i;
    //   break; 
    // }
    if (dist < dist_min) {
      dist_min = dist;
      index_min = i;
    }
  }

  if (len > 0) {
    float dist_sum = 0.0;
    int i = index_min+1;
    for (; i < path.poses.size(); i ++) {
      dist_sum += std::hypot(path.poses.at(i).pose.position.x - path.poses.at(i-1).pose.position.x,
                            path.poses.at(i).pose.position.y - path.poses.at(i-1).pose.position.y);
      if (dist_sum >= len) break;
    }
    if (dist_sum > 0) index_min = i;    
  } 

  carrot = path.poses.at(index_min);
  return true;
}



} // namespace edge_follower_ns
