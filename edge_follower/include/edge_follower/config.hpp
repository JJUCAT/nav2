#ifndef CONFIG_H
#define CONFIG_H

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "edge_follower/type.h"


namespace edge_follower_ns {

class Config
{
 public:
  Config(rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    const std::string name) : node_(node), name_(name) {
    logger_ = node_->get_logger();
  }
  ~Config() {}

  // 初始化配置
  // @nh: 节点句柄
  // @param: 返回的配置参数
  void LoadConfig(ef_params_t& params);

  // 更新 polygon 参数
  // @mode: 左/右沿边模式
  void UpdatePolygon(const ef_mode_t mode, ef_params_t& params);



 private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("EdgeFollowController")};
  std::string name_;
  ef_polygon_params_t polygon_;

};

} // namespace edge_follower_ns


#endif // CONFIG_H
