#include "edge_follow_controller/edge_follower.h"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include <memory>

namespace edge_follow_controller_ns
{

bool EdgeFollower::follow(nav2_costmap_2d::Costmap2D& map, const ef_dir_t dir,
  geometry_msgs::msg::Twist& cmd_vel)
{
  UpdateSteeringArea(dir);
  UpdateCheckTime(dir);
  return FollowAhead(cmd_vel);
}

// -------------------- private methods --------------------

void EdgeFollower::UpdateSteeringArea(const ef_dir_t dir)
{
  if (params_.mode == kLeft && dir == kDirR) {
    steering_left_ = params_.steering_adjust_limit;
    steering_right_ = -params_.steering_angle_max;
  } else if (params_.mode == kRight && dir == kDirL) {
    steering_left_ = params_.steering_angle_max;
    steering_right_ = -params_.steering_adjust_limit;
  } else {
    steering_left_ += params_.steering_adjust_open;
    steering_right_ -= params_.steering_adjust_open;
    steering_left_ = std::clamp(steering_left_,
      -params_.steering_angle_max, params_.steering_angle_max);
    steering_right_ = std::clamp(steering_right_,
      -params_.steering_angle_max, params_.steering_angle_max);
  }
}

void EdgeFollower::UpdateCheckTime(const ef_dir_t dir)
{
  if ((params_.mode == kLeft && dir == kDirR) ||
    (params_.mode == kRight && dir == kDirL)) {
    check_time_ = params_.avoid_check_time;
    check_time_step_ = params_.check_time_step * 2;
  } else {
    check_time_ -= 0.1;
    check_time_step_ -= 0.1;
    check_time_ = std::clamp(check_time_,
      params_.collision_check_time, params_.collision_check_time * 5);
    check_time_step_ = std::clamp(check_time_step_,
      params_.check_time_step, params_.check_time_step * 5);
  }
}


bool EdgeFollower::FollowAhead(geometry_msgs::msg::Twist& cmd_vel)
{
  float steering = 0.0f;
  geometry_msgs::msg::PolygonStamped follow_polygon, avoidance_polygon, viz_polygon;
  ef_dir_t dir = kDirN;
  float slow_approach;
  for (int i = 0; i < 2; i++) {
    // 沿边
    if (i == 0) {
      steering = last_steering_;
      if (params_.mode == kLeft) dir = kDirL;
      else dir = kDirR;
      follow_polygon.header.frame_id = costmap_ros_ptr_->getBaseFrameID();
      follow_polygon.polygon = params_.polygon_avoidance;
      // TransformPolygon(rbt_, follow_polygon, viz_polygon);
      // viz_polygon.header.frame_id = costmap_ros_ptr_->getGlobalFrameID();
      // sp_vizer_->VizPolygon(check_polygon_pub_, viz_polygon);
      slow_approach = 2 * params_.slow_approach;
      if (Close2(steering, follow_polygon, dir, slow_approach)) {
        // float speed = sp_speed_controller_->GetSpeedInLimit(steering, hz_);
        float speed = sp_speed_controller_->GetTurnSpeed(steering);
        SetCmdVel(cmd_vel, speed, steering);
        // SetLastCmdVel(speed, steering);
        return true;
      }
    }

    // 避障
    steering = last_steering_;
    if (params_.mode == kLeft) dir = kDirR;
    else dir = kDirL;
    if (i == 0) {
      avoidance_polygon.header.frame_id = costmap_ros_ptr_->getBaseFrameID();
      avoidance_polygon.polygon = params_.polygon_leave;
      // TransformPolygon(rbt_, avoidance_polygon, viz_polygon);
      // viz_polygon.header.frame_id = costmap_ros_ptr_->getGlobalFrameID();
      // sp_vizer_->VizPolygon(check_polygon_pub_, viz_polygon);
      slow_approach = 2 * params_.slow_approach;
    } else {
      avoidance_polygon.header.frame_id = costmap_ros_ptr_->getBaseFrameID();
      avoidance_polygon.polygon = params_.polygon;
      // TransformPolygon(rbt_, avoidance_polygon, viz_polygon);
      // viz_polygon.header.frame_id = costmap_ros_ptr_->getGlobalFrameID();
      // sp_vizer_->VizPolygon(check_polygon_pub_, viz_polygon);
      slow_approach = 2 * params_.slow_approach;
    }
    if (Close2(steering, avoidance_polygon, dir, slow_approach)) {
      // float speed = sp_speed_controller_->GetSpeedInLimit(steering, hz_);
      float speed = sp_speed_controller_->GetTurnSpeed(steering);
      SetCmdVel(cmd_vel, speed, steering);
      // SetLastCmdVel(speed, steering);
      return true;
    }
  }

  RCLCPP_INFO(logger_, "[%s,%d] can't find steering !", __FUNCTION__, __LINE__);
  return false;
}


} // namespace edge_follow_controller_ns
