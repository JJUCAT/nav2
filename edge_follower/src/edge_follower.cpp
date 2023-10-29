#include <edge_follower/edge_follower.h>
#include <limits>

namespace edge_follower_ns
{

EdgeFollower::EdgeFollower(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
  const ef_params_t* params)
  : params_(params)
{
  motion_ = std::make_shared<MotionAckerman>(params_);

  check_polygon_pub_ =
    node->create_publisher<geometry_msgs::msg::PolygonStamped>("ef_check_polygon", 1);
  avoidance_polygon_pub_ =
    node->create_publisher<geometry_msgs::msg::PolygonStamped>("ef_avoidance_polygon", 1);
  collision_cells_pub_ =
    node->create_publisher<sensor_msgs::msg::PointCloud>("collision_cells", 1);
  trajectorys_pub_ =
    node->create_publisher<visualization_msgs::msg::MarkerArray>("trajectorys", 1);
  lighthouse_pub_ =
    node->create_publisher<visualization_msgs::msg::Marker>("lighthouse", 1);

  check_time_ = params_->collision_check_time;
  check_time_step_ = params_->check_time_step;
  steering_left_ = params_->steering_angle_max;
  steering_right_ = -params_->steering_angle_max;
}

void EdgeFollower::findLighthouse(const nav_msgs::msg::Path& path, const unsigned int idx, const float len)
{
  float dist = 0;
  for (unsigned int i = idx, j = i +1; ; i ++, j ++) {
    if (j == path.poses.size()) j = 0;
    if (i == path.poses.size()) {i = 0; j = 1;}
    dist += (std::hypot(path.poses.at(j).pose.position.x - path.poses.at(i).pose.position.x,
                        path.poses.at(j).pose.position.y - path.poses.at(i).pose.position.y));
    if (dist >= len) {
      lighthouse_ = path.poses.at(j);
      break;
    }
  }
  // sp_vizer_->VizMarker(lighthouse_pub_, lighthouse_, visualization_msgs::Marker::SPHERE,
  //   0.3, 0.0, 0.0, 1.0, 1.0);
}

bool EdgeFollower::following(nav2_costmap_2d::Costmap2D* map,
  const ef_road_condition_t road_condition, geometry_msgs::msg::Twist& cmd_vel)
{
  UpdateSteeringArea(road_condition.dir);
  UpdateCheckTime(road_condition.dir);
  RCLCPP_INFO(logger_, "[EF], left:%f, right:%f, check_time:%f, check_step:%f !",
    steering_left_, steering_right_, check_time_, check_time_step_);
  geometry_msgs::msg::PolygonStamped follow_polygon, avoidance_polygon, viz_polygon;
  ef_dir_t dir = kDirN;
  std::vector<ef_traj_t> trajectorys;
  rclcpp::Time ct = rclcpp::Time();

  // 沿边
  if (params_->mode == kLeft) dir = kDirL;
  else dir = kDirR;
  follow_polygon.header.frame_id = "base_link";
  follow_polygon.polygon = params_->polygon_avoidance;
  GenerateTrajectory(map, cmd_vel, follow_polygon, dir, trajectorys);

  // 避障
  if (params_->mode == kLeft) dir = kDirR;
  else dir = kDirL;
  avoidance_polygon.header.frame_id = "base_link";
  if (use_leave_polygon_) avoidance_polygon.polygon = params_->polygon_leave;
  else avoidance_polygon.polygon = params_->polygon;
  GenerateTrajectory(map, cmd_vel, avoidance_polygon, dir, trajectorys);

  // 评估
  if (!trajectorys.empty()) {
    // ef_traj_t best_traj = FindBestTrajectory(trajectorys, cmd_vel.angular.z, map, road_condition.lighthouse);
    ef_traj_t best_traj = FindBestTrajectory(trajectorys, cmd_vel.angular.z, map, lighthouse_);
    // transform_polygon(robot_, avoidance_polygon, viz_polygon);
    // viz_polygon.header.frame_id = "map";
    // sp_vizer_->VizPolygon(check_polygon_pub_, viz_polygon);
    float speed = motion_->GetSpeed(cmd_vel.linear.x, best_traj.steering, road_condition.straight);
    // float speed = motion_->GetTurnSpeed(best_traj.steering);
    set_cmdvel(speed, best_traj.steering, cmd_vel);
    return true;
  }

  RCLCPP_WARN(logger_, "[EF], can't find steering !");
  return false;
}

// -------------------- private methods --------------------

void EdgeFollower::UpdateSteeringArea(const ef_dir_t dir)
{
  if (params_->mode == kLeft && dir == kDirR) {
    steering_left_ = params_->steering_adjust_limit;
    steering_right_ = -params_->steering_angle_max;
  } else if (params_->mode == kRight && dir == kDirL) {
    steering_left_ = params_->steering_angle_max;
    steering_right_ = -params_->steering_adjust_limit;
  } else {
    steering_left_ += params_->steering_adjust_open;
    steering_right_ -= params_->steering_adjust_open;
    steering_left_ = std::min(
      std::max(steering_left_, -params_->steering_angle_max),
      params_->steering_angle_max);
    steering_right_ = std::min(
      std::max(steering_right_, -params_->steering_angle_max),
      params_->steering_angle_max);
  }
}

void EdgeFollower::UpdateCheckTime(const ef_dir_t dir)
{
  if ((params_->mode == kLeft && dir == kDirR) || (params_->mode == kRight && dir == kDirL)) {
    check_time_ = params_->avoid_check_time;
    check_time_step_ = params_->check_time_step * 3;
  } else {
    check_time_ -= 0.1;
    check_time_step_ -= 0.05;
    check_time_ = std::min(
      std::max(check_time_, params_->collision_check_time),
      params_->collision_check_time * 5);
    check_time_step_ = std::min(
      std::max(check_time_step_, params_->check_time_step),
      params_->check_time_step * 5);
  }
}

size_t EdgeFollower::GenerateTrajectory(nav2_costmap_2d::Costmap2D* map, const geometry_msgs::msg::Twist& cmd_vel,
  geometry_msgs::msg::PolygonStamped& polygon, const ef_dir_t dir, std::vector<ef_traj_t>& trajectorys)
{
  float s = params_->sampling_step;
  float step = (dir == kDirL ? s : -s);
  float end = (dir == kDirL ? steering_left_ : steering_right_);
  float tmp_steering = cmd_vel.angular.z, tmp_speed;
  for (;; tmp_steering += step) {
    tmp_steering = std::min(
      std::max(tmp_steering, steering_right_), steering_left_);
    // tmp_speed = motion_->GetTurnSpeed(tmp_steering);
    tmp_speed = motion_->GetSpeed(cmd_vel.linear.x, tmp_steering, false);
    UpdateCheckPoses4Touch(tmp_steering, tmp_speed, check_time_step_, check_time_, params_->least_collision_check_dist);
    std::vector<nav2_costmap_2d::MapLocation> check_cells; 
    if (!isCollisionOnPoses(check_cells, collision_check_poses_, polygon, map)) {
      ef_traj_t trajectory;
      trajectory.steering = tmp_steering;
      trajectory.traj = collision_check_poses_;
      trajectorys.push_back(trajectory);
    }
    if (tmp_steering == end) break; 

    if (fabs(tmp_steering) < params_->sampling_boundary) {
      s = params_->sampling_tiny_step;
    } else {
      s = params_->sampling_step;
    }
    step = (dir == kDirL ? s : -s);
  }

  return trajectorys.size();
}

bool EdgeFollower::UpdateCheckPoses4Touch(const float steering_angle, const float linear_x,
    const float time_step, const float collision_check_time, const float least_collision_check_dist)
{
  collision_check_poses_.clear();
  if (linear_x == 0) {
    double dist_step = 0.4;
    for (double dist = 0; dist < least_collision_check_dist; dist += dist_step) {
      ef_point_t p = robot_;
      p.x = static_cast<float>(robot_.x + robot_.cos_theta * dist);
      p.y = static_cast<float>(robot_.y + robot_.sin_theta * dist);
      collision_check_poses_.push_back(p);
    }
  } else {
    float dist_sum = 0.f;
    collision_check_poses_.push_back(robot_);
    for (double t = time_step;; t += time_step) {
      if ((dist_sum > least_collision_check_dist) && (t > collision_check_time)) {
        break;
      }
      ef_point_t back = collision_check_poses_.back();
      ef_point_t p = motion_->PredictPose(robot_, steering_angle, linear_x, t);
      float angle_diff = get_diff_angle(robot_.theta, p.theta);
      if (fabs(angle_diff) > params_->check_angle_diff_max) {
        RCLCPP_DEBUG(logger_, "[EF] pose angle diff:%f is larger than max angle diff:%f.",
          angle_diff, params_->check_angle_diff_max);
        break;
      }
      dist_sum += std::hypot(p.x - back.x, p.y - back.y);
      collision_check_poses_.push_back(p);
    }
  }

  std::vector<ef_point_t> pl(collision_check_poses_.begin(), collision_check_poses_.end());
  // sp_vizer_->VizCheckPoses(checks_poses_pub_, pl);
  return true;
}

float EdgeFollower::ScoreClose(const ef_traj_t& trajectory)
{
  float steering = trajectory.steering;
  if (params_->mode == kLeft) steering = steering + params_->steering_angle_max;
  else steering = params_->steering_angle_max - steering;
  float score = steering * 100.f / (params_->steering_angle_max * 2);
  return score;
}

float EdgeFollower::ScoreOscillation(const ef_traj_t& trajectory, const float last_steering)
{
  auto isOscillation = [](const float cur_diff, const float last_diff) {
    float err = 0.085;
    if (fabs(cur_diff) < err) return false;
    if ((cur_diff < -err && last_diff < -err) ||
        (cur_diff > err && last_diff > err)) return false;
    return true;
  };

  float dist = std::hypot(robot_.x - trajectory.traj.back().x,
                          robot_.y - trajectory.traj.back().y);
  float sdiff = trajectory.steering - last_steering;
  for (int i = history_.size()-1; i > 0; i --) {
    float tdiff = history_.at(i).steering - history_.at(i-1).steering;
    if (isOscillation(sdiff, tdiff)) break;
    dist += std::hypot(history_.at(i).point.x - history_.at(i-1).point.x,
                       history_.at(i).point.y - history_.at(i-1).point.y);    
  }
  
  float diff = fabs(trajectory.steering - last_steering);
  if (dist < 3.f) diff *= 1.2;
  float score = (params_->steering_angle_max * 2 - diff) * 100.f / 
    (params_->steering_angle_max * 2);
  return score;
}

float EdgeFollower::ScoreObstacle(const ef_traj_t& trajectory,
  nav2_costmap_2d::Costmap2D* map, const unsigned char less_cost)
{
  float total_score = 0.f;
  for (unsigned int i = 0; i < trajectory.traj.size(); i ++) {
    ef_point_t head_pose = trajectory.traj.at(i);
    head_pose.x = head_pose.x + head_pose.cos_theta * params_->wheel_base;
    head_pose.y = head_pose.y + head_pose.sin_theta * params_->wheel_base;

    nav2_costmap_2d::MapLocation map_point;
    if (!map->worldToMap(head_pose.x, head_pose.y, map_point.x, map_point.y)) {
      RCLCPP_WARN(logger_, "[EF] Polygon point [%f,%f] lies outside map bounds !",
        head_pose.x, head_pose.y);
      return 0.0;
    }
    unsigned char cost = map->getCost(map_point.x, map_point.y);
    cost = cost > nav2_costmap_2d::LETHAL_OBSTACLE ? nav2_costmap_2d::LETHAL_OBSTACLE : cost;
    cost = cost < less_cost ? less_cost : cost;
    total_score += static_cast<float>(nav2_costmap_2d::LETHAL_OBSTACLE - cost) * 100.f /
                   nav2_costmap_2d::LETHAL_OBSTACLE;
  }
  float score = total_score / trajectory.traj.size();
  return score;
}

float EdgeFollower::ScoreAlign(const ef_traj_t& trajectory, const geometry_msgs::msg::PoseStamped lighthouse)
{
  float target_steering = motion_->PurepursuitFollow(robot_pose_, lighthouse,
    params_->wheel_base);
  target_steering = std::min(
    std::max(target_steering,-params_->steering_angle_max),
    params_->steering_angle_max);
  float diff = fabs(trajectory.steering - target_steering);
  float score = (params_->steering_angle_max * 2 - diff) * 100.f / 
    (params_->steering_angle_max * 2);
  // ROS_INFO("[EF ROS][%s,%d] target_steering:%f, score:%f",
  //   __FUNCTION__, __LINE__, target_steering, score);
  return score;
}

ef_traj_t EdgeFollower::FindBestTrajectory(
  const std::vector<ef_traj_t>& trajectorys, const float last_steering,
  nav2_costmap_2d::Costmap2D* map, const geometry_msgs::msg::PoseStamped lighthouse)
{
  std::vector<ef_critic_t> critics;
  int best_idx = -1;
  float score = std::numeric_limits<float>::lowest();
  for (unsigned int i = 0; i < trajectorys.size(); i ++) {
    ef_critic_t critic;
    critic.score_close = ScoreClose(trajectorys.at(i));
    critic.score_oscillation = ScoreOscillation(trajectorys.at(i), last_steering);
    critic.score_obstacle = ScoreObstacle(trajectorys.at(i), map);
    critic.score_align = ScoreAlign(trajectorys.at(i), lighthouse);
    critic.score = critic.score_close * params_->weight_close +
                   critic.score_oscillation * params_->weight_oscillation +
                   critic.score_obstacle * params_->weight_obstacle +
                   critic.score_align * params_->weight_align;
    critics.push_back(critic);
    // ROS_INFO("[EF ROS][%s,%d] traj[%d]:%f", __FUNCTION__, __LINE__, i, critic.score);
    if (critic.score > score) {
      score = critic.score;
      best_idx = i;
    }

    // ROS_INFO("[EF ROS][%s,%d] trajectory[%d][%f], score_close:%f, score_oscillation:%f,
    // score_obstacle:%f, score_align:%f, final_score:%f",
    //   __FUNCTION__, __LINE__, i, trajectorys.at(i).steering, critic.score_close,
    //   critic.score_oscillation, critic.score_obstacle, critic.score_align, critic.score); 
  }

  // sp_vizer_->VizTrajectorys(trajectorys_pub_, "map",
  //   trajectorys, critics, critics.at(best_idx).score, 
  //   visualization_msgs::Marker::ARROW, 0.05f,
  //   1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f);
  history_.push_back(ef_sample_point_t(trajectorys.at(best_idx).steering, robot_));
  if (history_.size() > 20) history_.erase(history_.begin());
  RCLCPP_WARN(logger_, "[EF] best trajectory[%d] score:%f, steering:%f.",
    best_idx, score, trajectorys.at(best_idx).steering);
  return trajectorys.at(best_idx);
}

} // namespace edge_follower_ns
