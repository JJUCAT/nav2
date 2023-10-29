#ifndef EDGE_FOLLOWER_H
#define EDGE_FOLLOWER_H


#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <edge_follower/type.h>
#include <edge_follower/common.h>
#include <edge_follower/motion.h>
#include <edge_follower/nanoflann_port.h>

namespace edge_follower_ns {


class EdgeFollower
{
 public:
  EdgeFollower(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    const ef_params_t* params);

  ~EdgeFollower() {}
  
  void updateMap(nav2_costmap_2d::Costmap2D* map) { map_ = map; }

  void updateRobotPose(const geometry_msgs::msg::PoseStamped pose, const ef_point_t efp) {
    robot_pose_ = pose;
    robot_ = efp;
  }

  void useLeavePolygon(const bool use) { use_leave_polygon_ = use; }

  void findLighthouse(const nav_msgs::msg::Path& path, const unsigned int idx, const float len);

  bool following(nav2_costmap_2d::Costmap2D* map,
    const ef_road_condition_t road_condition, geometry_msgs::msg::Twist& cmd_vel);

 private:
  void UpdateCheckTime(const ef_dir_t dir);

  void UpdateSteeringArea(const ef_dir_t dir);

  size_t GenerateTrajectory(nav2_costmap_2d::Costmap2D* map, const geometry_msgs::msg::Twist& cmd_vel,
    geometry_msgs::msg::PolygonStamped& polygon, const ef_dir_t dir, std::vector<ef_traj_t>& trajectorys);

  bool UpdateCheckPoses4Touch(const float steering_angle, const float linear_x,
    const float time_step, const float collision_check_time, const float least_collision_check_dist);

  bool IsCollisionImminent(const geometry_msgs::msg::PolygonStamped& polygon,
    std::vector<nav2_costmap_2d::MapLocation>& cells);

  // 检查碰撞检测框是否在障碍物中
  bool IsCollisionInPolygon(
    const geometry_msgs::msg::PolygonStamped& check_polygon,
    std::vector<nav2_costmap_2d::MapLocation>& cells,
    const unsigned char collision_value = nav2_costmap_2d::LETHAL_OBSTACLE);

  float ScoreClose(const ef_traj_t& trajectory);
  float ScoreOscillation(const ef_traj_t& trajectory, const float last_steering);
  /**
   * @brief 沿边采样评分器，障碍物评分
   * 
   * @param trajectory 采样轨迹，轨迹是无碰撞的
   * @param map        代价地图
   * @param less_cost  认为的代价地图最低代价值，避免转弯时候偏向空旷处
   * @return float 
   */
  float ScoreObstacle(const ef_traj_t& trajectory, nav2_costmap_2d::Costmap2D* map, const unsigned char less_cost = 200);
  float ScoreAlign(const ef_traj_t& trajectory, const geometry_msgs::msg::PoseStamped lighthouse);
  ef_traj_t FindBestTrajectory(const std::vector<ef_traj_t>& trajectorys,
    const float last_steering, nav2_costmap_2d::Costmap2D* map, const geometry_msgs::msg::PoseStamped lighthouse);
  ef_traj_t VizTrajectorys(
    const std::vector<ef_traj_t>& trajectorys, const std::vector<ef_critic_t>& critics);

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PolygonStamped>::SharedPtr check_polygon_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PolygonStamped>::SharedPtr avoidance_polygon_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud>::SharedPtr collision_cells_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectorys_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr lighthouse_pub_;

  rclcpp::Logger logger_{rclcpp::get_logger("EdgeFollower")};
  std::shared_ptr<Motion> motion_;
  geometry_msgs::msg::PoseStamped robot_pose_;
  ef_point_t robot_;  
  nav2_costmap_2d::Costmap2D* map_;
  const ef_params_t* params_;
  ef_dir_t dir_;
  float check_time_;
  float check_time_step_;
  float steering_left_, steering_right_;
  std::vector<ef_point_t> collision_check_poses_;
  std::vector<nav2_costmap_2d::MapLocation> collision_cells_;
  bool use_leave_polygon_ = false;
  std::vector<ef_sample_point_t> history_;
  geometry_msgs::msg::PoseStamped lighthouse_;
};

} // namespace edge_follower_ns

#endif // EDGE_FOLLOWER_H

