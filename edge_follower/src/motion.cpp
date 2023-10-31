#include <edge_follower/motion.h>
#include <tf2/utils.h>

namespace edge_follower_ns {

float MotionAckerman::GetTurnSpeed(const float steering)
{
  float speed_spin = params_->max_speed - params_->min_speed;
  float percent = exp(-abs(steering) * params_->speed_scaling_factor);
  float linear_x = speed_spin * percent + params_->min_speed; 
  // ROS_DEBUG("[%s,%d] linear_x:%f", __FUNCTION__, __LINE__, linear_x);
  return linear_x;
}

float MotionAckerman::GetSpeed(const float cur_speed, const float cur_steering,
  const bool is_straight)
{
  float speed = cur_speed;
  if (is_straight) {
    float scale = 1.f - (fabs(cur_steering) / params_->steering_angle_max);
    speed += scale * params_->acc_max / params_->control_hz;
  } else {
    float scale = fabs(cur_steering) / params_->steering_angle_max;
    speed -= scale * params_->dec_max / params_->control_hz;
  }
  // ROS_INFO("[%s,%d] get speed, cur_speed:%f, speed:%f, min:%f, max:%f",
  //   __FUNCTION__, __LINE__, cur_speed, speed, params_->min_speed, params_->max_speed);
  speed = std::min(std::max(speed, params_->min_speed), params_->max_speed);
  return speed;
}

ef_point_t MotionAckerman::PredictPose(const ef_point_t& pose,
    const float steering, const float linear_x, const double time)
{
  double delta_x = 0;
  double delta_y = 0;
  double beta = 0;
  if (abs(steering) < 0.017) {
      delta_x = linear_x * time;
  } else {
      double r = params_->wheel_base / tan(steering);
      beta = time * linear_x / r;
      delta_x = sin(beta) * r;
      delta_y = r * (1 - cos(beta));
  }

  ef_point_t p;
  p.x = static_cast<float>(pose.x + pose.cos_theta * delta_x - pose.sin_theta * delta_y);
  p.y = static_cast<float>(pose.y + pose.sin_theta * delta_x + pose.cos_theta * delta_y);
  p.theta = pose.theta + beta;
  if (p.theta < -M_PI)
      p.theta += 2 * M_PI;
  else if (p.theta > M_PI)
      p.theta -= 2 * M_PI;
  p.sin_theta = sin(p.theta);
  p.cos_theta = cos(p.theta);
  return p;
}

float MotionAckerman::StanleyFollow(
  const geometry_msgs::msg::PoseStamped steering_pose,
  const geometry_msgs::msg::PoseStamped target,
  const float mu, const float lambda,
  const float ks, const float linear_x)
{
  float steer_yaw = tf2::getYaw(steering_pose.pose.orientation);
  float target_yaw = tf2::getYaw(target.pose.orientation);
  float yaw_err =  mu * get_diff_angle(steer_yaw, target_yaw);  
  float dist_diff = std::hypot(
    target.pose.position.y - steering_pose.pose.position.y,
    target.pose.position.x - steering_pose.pose.position.x);
  float relative_yaw = atan2(target.pose.position.y - steering_pose.pose.position.y,
                             target.pose.position.x - steering_pose.pose.position.x);
  if (get_diff_angle(steer_yaw, relative_yaw) < 0) dist_diff = -dist_diff;
  // ROS_INFO("[%s,%d] relative_yaw:%f, steer_yaw:%f",
  //   __FUNCTION__, __LINE__, relative_yaw, steer_yaw);
  float dist_err = atan(lambda * dist_diff / (linear_x + ks));
  float steering = yaw_err + dist_err;
  // ROS_INFO("[%s,%d] yaw_err:%f, dist_diff:%f, dist_err:%f, steering:%f",
  //   __FUNCTION__, __LINE__, yaw_err, dist_diff, dist_err, steering);
  return steering;
}

float MotionAckerman::PurepursuitFollow(
  const geometry_msgs::msg::PoseStamped robot, const geometry_msgs::msg::PoseStamped target,
  const float wheel_base)
{
  float dist = std::hypot(target.pose.position.y - robot.pose.position.y,
                           target.pose.position.x - robot.pose.position.x);
  float target_dir = atan2(target.pose.position.y - robot.pose.position.y,
                           target.pose.position.x - robot.pose.position.x);
  float robot_dir = tf2::getYaw(robot.pose.orientation);
  float alpha = get_diff_angle(robot_dir, target_dir);
  float steering = atan(2 * wheel_base * sin(fabs(alpha)) / dist);
  if (alpha > 0) steering = fabs(steering);
  else if (alpha < 0) steering = -fabs(steering);
  return steering;
}


} // namespace edge_follower_ns
