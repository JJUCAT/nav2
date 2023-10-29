#ifndef MOTION_H
#define MOTION_H

#include <edge_follower/type.h>
#include <edge_follower/common.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace edge_follower_ns
{

  class Motion
  {
   public:
    explicit Motion(const ef_params_t* params)
      : params_(params) {} 
    ~Motion() {}
    
    virtual float GetTurnSpeed(const float steering_angular) = 0;
    
    virtual float GetSpeed(const float cur_speed, const float cur_steering,
      const bool is_straight) = 0;

    virtual ef_point_t PredictPose(const ef_point_t& pose,
        const float steering_angular, const float linear_x, const double time) = 0;

    virtual float StanleyFollow(
      const geometry_msgs::msg::PoseStamped steering_angular,
      const geometry_msgs::msg::PoseStamped target,
      const float wheel_base, const float mu, const float lambda,
      const float ks, const float linear_x) = 0;

    virtual float PurepursuitFollow(
      const geometry_msgs::msg::PoseStamped robot,
      const geometry_msgs::msg::PoseStamped target,
      const float wheel_base) = 0;

   protected:
    const ef_params_t* params_;

  }; // class Motion


  class MotionAckerman : public Motion
  {
   public:
    explicit MotionAckerman(const ef_params_t* params)
      : Motion(params)  {} 
    ~MotionAckerman() {}
    
    float GetTurnSpeed(const float steering) override;

    float GetSpeed(const float cur_speed, const float cur_steering,
      const bool is_straight) override;

    ef_point_t PredictPose(const ef_point_t& pose,
        const float steering, const float linear_x, const double time) override;

    virtual float StanleyFollow(
      const geometry_msgs::msg::PoseStamped steering_pose,
      const geometry_msgs::msg::PoseStamped target,
      const float wheel_base, const float mu, const float lambda,
      const float ks, const float linear_x) override;
    
    virtual float PurepursuitFollow(
      const geometry_msgs::msg::PoseStamped robot,
      const geometry_msgs::msg::PoseStamped target,
      const float wheel_base) override;

   private:

  }; // class MotionAckerman




} // namespace edge_follower_ns

#endif // MOTION_H
