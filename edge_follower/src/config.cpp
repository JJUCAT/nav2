#include "edge_follower/config.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_core/exceptions.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace edge_follower_ns {

void Config::LoadConfig(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
  ef_params_t& params)
{
  declare_parameter_if_not_declared(
    node, name_ + ".mode", rclcpp::ParameterValue(1.0));
  node->get_parameter(name_ + ".mode", params.mode);

  declare_parameter_if_not_declared(
    node, name_ + ".timeout", rclcpp::ParameterValue(5.0));
  node->get_parameter(name_ + ".timeout", params.timeout);

  declare_parameter_if_not_declared(
    node, name_ + ".map_update_hz", rclcpp::ParameterValue(2.0));
  node->get_parameter(name_ + ".map_update_hz", params.map_update_hz);

  declare_parameter_if_not_declared(
    node, name_ + ".head", rclcpp::ParameterValue(1.5));
  node->get_parameter(name_ + ".head", params.head);


  declare_parameter_if_not_declared(
    node, name_ + ".steering_adjust_limit", rclcpp::ParameterValue(0.5));
  node->get_parameter(name_ + ".steering_adjust_limit", params.steering_adjust_limit);

  declare_parameter_if_not_declared(
    node, name_ + ".steering_adjust_close", rclcpp::ParameterValue(0.17));
  node->get_parameter(name_ + ".steering_adjust_close", params.steering_adjust_close);

  declare_parameter_if_not_declared(
    node, name_ + ".steering_adjust_open", rclcpp::ParameterValue(0.17));
  node->get_parameter(name_ + ".steering_adjust_open", params.steering_adjust_open);

  declare_parameter_if_not_declared(
    node, name_ + ".sampling_step", rclcpp::ParameterValue(0.051));
  node->get_parameter(name_ + ".sampling_step", params.sampling_step);

  declare_parameter_if_not_declared(
    node, name_ + ".sampling_boundary", rclcpp::ParameterValue(0.17));
  node->get_parameter(name_ + ".sampling_boundary", params.sampling_boundary);

  declare_parameter_if_not_declared(
    node, name_ + ".sampling_tiny_step", rclcpp::ParameterValue(0.17));
  node->get_parameter(name_ + ".sampling_tiny_step", params.sampling_tiny_step);

  declare_parameter_if_not_declared(
    node, name_ + ".slow_approach", rclcpp::ParameterValue(0.051));
  node->get_parameter(name_ + ".slow_approach", params.slow_approach);

  declare_parameter_if_not_declared(
    node, name_ + ".preview_dist", rclcpp::ParameterValue(8.0));
  node->get_parameter(name_ + ".preview_dist", params.preview_dist);

  declare_parameter_if_not_declared(
    node, name_ + ".preview_res", rclcpp::ParameterValue(0.2));
  node->get_parameter(name_ + ".preview_res", params.preview_res);

  declare_parameter_if_not_declared(
    node, name_ + ".dir_ahead_angle", rclcpp::ParameterValue(0.51));
  node->get_parameter(name_ + ".dir_ahead_angle", params.dir_ahead_angle);

  declare_parameter_if_not_declared(
    node, name_ + ".limit_hz", rclcpp::ParameterValue(1.0));
  node->get_parameter(name_ + ".limit_hz", params.limit_hz);

  declare_parameter_if_not_declared(
    node, name_ + ".min_speed", rclcpp::ParameterValue(0.1));
  node->get_parameter(name_ + ".min_speed", params.min_speed);

  declare_parameter_if_not_declared(
    node, name_ + ".max_speed", rclcpp::ParameterValue(0.5));
  node->get_parameter(name_ + ".max_speed", params.max_speed);

  declare_parameter_if_not_declared(
    node, name_ + ".max_angular", rclcpp::ParameterValue(0.34));
  node->get_parameter(name_ + ".max_angular", params.max_angular);

  declare_parameter_if_not_declared(
    node, name_ + ".angular_acc_max", rclcpp::ParameterValue(0.17));
  node->get_parameter(name_ + ".angular_acc_max", params.angular_acc_max);

  declare_parameter_if_not_declared(
    node, name_ + ".angular_dec_max", rclcpp::ParameterValue(-0.34));
  node->get_parameter(name_ + ".angular_dec_max", params.angular_dec_max);

  declare_parameter_if_not_declared(
    node, name_ + ".speed_scaling_factor", rclcpp::ParameterValue(3.0));
  node->get_parameter(name_ + ".speed_scaling_factor", params.speed_scaling_factor);

  declare_parameter_if_not_declared(
    node, name_ + ".acc_max", rclcpp::ParameterValue(0.5));
  node->get_parameter(name_ + ".acc_max", params.acc_max);

  declare_parameter_if_not_declared(
    node, name_ + ".dec_max", rclcpp::ParameterValue(0.5));
  node->get_parameter(name_ + ".dec_max", params.dec_max);

  declare_parameter_if_not_declared(
    node, name_ + ".steering_angle_max", rclcpp::ParameterValue(1.0));
  node->get_parameter(name_ + ".steering_angle_max", params.steering_angle_max);

  node->declare_parameter("foo");
  rclcpp::Parameter foo_param("foo", std::vector<double>({}));

  rclcpp::Parameter check_polygon;
  node->get_parameter(name_ + ".check_polygon", check_polygon);
  if (check_polygon.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
    std::vector<double> polygon = check_polygon.as_double_array();
    for (unsigned int i = 0; i < polygon.size()-1; i += 2) {
      double x = polygon.at(i);
      double y = polygon.at(i+1);
      geometry_msgs::msg::Point32 p;
      p.x = static_cast<float>(x);
      p.y = static_cast<float>(y);
      params.polygon.points.push_back(p);
    }
    polygon_.polygon = params.polygon;
  } else {
    RCLCPP_INFO(logger_, "[%s,%d] polygon error !",
      __FUNCTION__, __LINE__);
  }

  rclcpp::Parameter avoidance_polygon;
  node->get_parameter(name_ + ".avoidance_polygon", check_polygon);
  if (avoidance_polygon.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
    std::vector<double> polygon = avoidance_polygon.as_double_array();
    for (unsigned int i = 0; i < polygon.size()-1; i += 2) {
      double x = polygon.at(i);
      double y = polygon.at(i+1);
      geometry_msgs::msg::Point32 p;
      p.x = static_cast<float>(x);
      p.y = static_cast<float>(y);
      params.polygon_avoidance.points.push_back(p);
    }
    polygon_.polygon_avoidance = params.polygon_avoidance;
  } else {
    RCLCPP_INFO(logger_, "[%s,%d] polygon error !",
      __FUNCTION__, __LINE__);
  }

  rclcpp::Parameter leave_polygon;
  node->get_parameter(name_ + ".leave_polygon", leave_polygon);
  if (leave_polygon.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
    std::vector<double> polygon = leave_polygon.as_double_array();
    for (unsigned int i = 0; i < polygon.size()-1; i += 2) {
      double x = polygon.at(i);
      double y = polygon.at(i+1);
      geometry_msgs::msg::Point32 p;
      p.x = static_cast<float>(x);
      p.y = static_cast<float>(y);
      params.polygon_leave.points.push_back(p);
    }

    polygon_.polygon_leave = params.polygon_leave;
  } else {
    RCLCPP_INFO(logger_, "[%s,%d] polygon error !",
      __FUNCTION__, __LINE__);
  }

  rclcpp::Parameter horizon_polygon;
  node->get_parameter(name_ + ".horizon_polygon", horizon_polygon);
  if (horizon_polygon.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
    std::vector<double> polygon = horizon_polygon.as_double_array();
    for (unsigned int i = 0; i < polygon.size()-1; i += 2) {
      double x = polygon.at(i);
      double y = polygon.at(i+1);
      geometry_msgs::msg::Point32 p;
      p.x = static_cast<float>(x);
      p.y = static_cast<float>(y);
      params.polygon_horizon.points.push_back(p);
    }

    polygon_.polygon_horizon = params.polygon_horizon;
  } else {
    RCLCPP_INFO(logger_, "[%s,%d] polygon error !",
      __FUNCTION__, __LINE__);
  }

  rclcpp::Parameter horizon_polygon_plus;
  node->get_parameter(name_ + ".horizon_polygon_plus", horizon_polygon);
  if (horizon_polygon_plus.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
    std::vector<double> polygon = horizon_polygon_plus.as_double_array();
    for (unsigned int i = 0; i < polygon.size()-1; i += 2) {
      double x = polygon.at(i);
      double y = polygon.at(i+1);
      geometry_msgs::msg::Point32 p;
      p.x = static_cast<float>(x);
      p.y = static_cast<float>(y);
      params.polygon_horizon_plus.points.push_back(p);
    }

    polygon_.polygon_horizon_plus = params.polygon_horizon_plus;
  } else {
    RCLCPP_INFO(logger_, "[%s,%d] polygon error !",
      __FUNCTION__, __LINE__);
  }

  declare_parameter_if_not_declared(
    node, name_ + ".check_angle_diff_max", rclcpp::ParameterValue(1.05));
  node->get_parameter(name_ + ".check_angle_diff_max", params.check_angle_diff_max);

  declare_parameter_if_not_declared(
    node, name_ + ".check_time_step", rclcpp::ParameterValue(0.5));
  node->get_parameter(name_ + ".check_time_step", params.check_time_step);

  declare_parameter_if_not_declared(
    node, name_ + ".collision_check_time", rclcpp::ParameterValue(2.0));
  node->get_parameter(name_ + ".collision_check_time", params.collision_check_time);

  declare_parameter_if_not_declared(
    node, name_ + ".avoid_check_time", rclcpp::ParameterValue(4.0));
  node->get_parameter(name_ + ".avoid_check_time", params.avoid_check_time);

  declare_parameter_if_not_declared(
    node, name_ + ".least_collision_check_dist", rclcpp::ParameterValue(0.5));
  node->get_parameter(name_ + ".least_collision_check_dist", params.least_collision_check_dist);

  declare_parameter_if_not_declared(
    node, name_ + ".ref_path_check_radius", rclcpp::ParameterValue(0.8));
  node->get_parameter(name_ + ".ref_path_check_radius", params.ref_path_check_radius);

  declare_parameter_if_not_declared(
    node, name_ + ".ref_path_check_step", rclcpp::ParameterValue(0.5));
  node->get_parameter(name_ + ".ref_path_check_step", params.ref_path_check_step);

  declare_parameter_if_not_declared(
    node, name_ + ".ref_path_check_len", rclcpp::ParameterValue(3.5));
  node->get_parameter(name_ + ".ref_path_check_len", params.ref_path_check_len);
}

void Config::UpdatePolygon(const ef_mode_t mode, ef_params_t& params)
{
  // 右沿边用原始参数
  params.polygon = polygon_.polygon;
  params.polygon_avoidance = polygon_.polygon_avoidance;
  params.polygon_horizon = polygon_.polygon_horizon;
  params.polygon_horizon_plus = polygon_.polygon_horizon_plus;

  // 左沿边参数
  if (mode == kLeft) {
    for (unsigned int i = 0; i < params.polygon.points.size(); i ++) {
      params.polygon.points.at(i).y *= -1;
    }

    for (unsigned int i = 0; i < params.polygon_avoidance.points.size(); i ++) {
      params.polygon_avoidance.points.at(i).y *= -1;
    }

    for (unsigned int i = 0; i < params.polygon_horizon.points.size(); i ++) {
      params.polygon_horizon.points.at(i).y *= -1;
    }

    for (unsigned int i = 0; i < params.polygon_horizon_plus.points.size(); i ++) {
      params.polygon_horizon_plus.points.at(i).y *= -1;
    }
  }
}


} // namespace edge_follower_ns

