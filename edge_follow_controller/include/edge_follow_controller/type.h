#ifndef TYPE_H
#define TYPE_H

#include "geometry_msgs/msg/polygon.hpp"

namespace edge_follow_controller_ns {

  typedef enum {
    kLeft, // 机器左侧沿边
    kRight, // 机器右侧沿边
  } ef_mode_t;

  typedef enum {
    kDirN, // 丢失方向
    kDirL, // 左转
    kDirR, // 右转
    kDirS, // 直行
  } ef_dir_t;

  // ef_params_t 中 polygon 的原始参数
  // 这个参数为右沿边模式下的参数，左沿边只需要 y 值取反
  typedef struct {
    geometry_msgs::msg::Polygon polygon; // 碰撞检测框
    geometry_msgs::msg::Polygon polygon_avoidance; // 碰撞检测框
    geometry_msgs::msg::Polygon polygon_leave; // 碰撞检测框
    geometry_msgs::msg::Polygon polygon_horizon; // 视野检测框，用于判断前方路径方向
    geometry_msgs::msg::Polygon polygon_horizon_plus; // 视野检测框，用于判断前方路径方向
  } ef_polygon_params_t;

  // 加载的参数，运行时使用
  // 其中的 polygon 参数在运行时候检查‘ef_params_t.mode’模式做调整
  // polygon 的原始参数会保存在 ef_polygon_params_t 中
  typedef struct {
    int mode;  // 沿边模式，@ef_mode_t
    float timeout; // 沿边超时时间，uint:s
    float map_update_hz; // 地图处理频率
    float opt_map_size; // 地图大小，uint:m
    float dilate_radius; // 障碍物膨胀半径用于提取轮廓，uint:m
    float head; // 判断参考路线时候，车头的位置，uint:m
    float dilate_edge; // 膨胀障碍物，用于控制沿边距离，uint:m
    bool pass_unknown; // 是否穿越未知区域
    float steering_adjust_limit; // 转角范围的调整，关闭的那侧限制角度，uint:rad
    float steering_adjust_close; // 转角范围调整，关闭时候的步长，uint:rad
    float steering_adjust_open; // 转角范围调整，打开时候的步长，uint:rad
    float sampling_step; // 粗糙预测时候，前轮角度的检查步进，uint:rad
    float sampling_boundary; // 采样步进从 <step> 到 <tiny step> 的分界线，uint:rad
    float sampling_tiny_step; // 采样前轮角度的步进，uint:rad
    float interspace; // 历史轨迹点间距，uint:m
    float sparse_interspace; // 检查沿边是否闭合的稀疏轨迹点间距，uint:m
    float dist_err4close; // 检查沿边闭合点的距离容错误差，uint:m
    float yaw_err4close; // 检查沿边闭合点的角度容差误差，uint:rad
    float contours_limit_len; // 轮廓最短长度，uint:m
    float slow_approach; // 慢慢靠近，uint:rad
    float preview_dist; // 参考方向的轮廓提取距离，uint:m
    float preview_res; // 参考路径的点间距，uint:m
    float dir_ahead_angle; // 参考方向“前方”的角度阈值，uint:rad
    float axis; // 轮距
    float arm; // 轮距中心到前轮的距离
    float limit_hz; // 最低工作频率
    float min_speed; // 最小速度
    float max_speed; // 最大速度
    float max_angular; // 最大角速度
    float angular_acc_max; // 最大角加速度
    float angular_dec_max; // 最小角加速度
    float speed_scaling_factor; //速度调节系数
    float acc_max; // 最大加速度
    float dec_max; // 最大减速度
    float steering_angle_max; // 前轮最大转角
    geometry_msgs::msg::Polygon polygon; // 碰撞检测框
    geometry_msgs::msg::Polygon polygon_avoidance; // 碰撞检测框
    geometry_msgs::msg::Polygon polygon_leave; // 碰撞检测框
    geometry_msgs::msg::Polygon polygon_horizon; // 视野检测框，用于判断前方路径方向
    geometry_msgs::msg::Polygon polygon_horizon_plus; // 视野检测框，用于判断前方路径方向
    std::string odom_topic; // 获取机器速度的话题
    float check_angle_diff_max; // 采样时候最大角度差
    float check_time_step; // 检测的时间步长
    float collision_check_time; // 碰撞检测预测时间
    float avoid_check_time; // 避障检测预测时间
    float least_collision_check_dist; // 碰撞检测预测最短距离
    float hook; // 跟随模式下，车头跟随点位置，uint:m
    float ref_path_check_radius; // 参考路径检测上的检测半径，uint:m
    float ref_path_check_step; // 参考路径检测步进，uint:m
    float ref_path_check_len; // 参考路径检测长度，uint:m
    bool cling;
  } ef_params_t;

  typedef struct {
    float x;
    float y;
    float theta;
    float sin_theta;
    float cos_theta;
  } ef_point_t;

  const unsigned char kBoundary = 233;
  const unsigned char kWall = 232;

  typedef struct {
    bool get_info;
    double width;
    double height;
    double resolution;
    double origin_x;
    double origin_y;
  } ef_map_info_t;
} // namespace edge_follower_ns


#endif // TYPE_H
