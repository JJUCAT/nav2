#include <edge_follower/common.h>
#include <limits>

namespace edge_follower_ns {


float get_diff_angle(const float angle_start, const float angle_end)
{
  float diff_angle = angle_end - angle_start;
  if (diff_angle > M_PI) {
    diff_angle = diff_angle - 2 * M_PI;
  } else if (diff_angle < -M_PI) {
    diff_angle = 2 * M_PI + diff_angle;
  }
  return diff_angle;
}


bool in_polygon(const geometry_msgs::msg::Polygon& polygon, const geometry_msgs::msg::Point p)
{
  bool in = false;
  for (unsigned int i = 0, j = polygon.points.size() - 1; i < polygon.points.size(); j = i, i++) {
    geometry_msgs::msg::Point32 start = polygon.points.at(i), end = polygon.points.at(j);
    // 点与多边形顶点重合
    if ((start.x == p.x && start.y == p.y) || (end.x == p.x && end.y == p.y))
      return true;

    // 判断线段两端点是否在射线两侧
    if ((start.y < p.y && end.y >= p.y) || (start.y >= p.y && end.y < p.y)) {
      // 线段上与射线 Y 坐标相同的点的 X 坐标
      float x = start.x + (p.y - start.y) * (end.x - start.x) / (end.y - start.y);

      // 点在多边形的边上
      if (x == p.x) return true;

      // 射线穿过多边形的边界
      if (x > p.x) in = !in;
    }
  }

  return in;
}

void create_circle_polygon(geometry_msgs::msg::PolygonStamped& polygon,
  const float radius)
{
  polygon.header.frame_id = "base_link";
  polygon.header.stamp = rclcpp::Time();
  polygon.polygon.points.clear();
  geometry_msgs::msg::Point32 p32;
  for (float a = 0; a <= 6.28; a += 0.51) {
    p32.x = radius * cos(a);
    p32.y = radius * sin(a);
    polygon.polygon.points.push_back(p32);
  }
}

void create_ellipse_polygon(geometry_msgs::msg::PolygonStamped& polygon,
  const float a, const float b, const float offset)
{
  auto GetY = [=](const float x) { return sqrt((1-(x*x)/(b*b))*(a*a)); };
  // auto GetX = [=](const float y) { return sqrt((1-(y*y)/(a*a))*(b*b)); };
  polygon.header.frame_id = "base_link";
  polygon.header.stamp = rclcpp::Time();
  polygon.polygon.points.clear();
  float d = 0.001, axis = b;
  geometry_msgs::msg::Point32 p32; // nan 2, 6
  p32.x = 0.0f; p32.y = GetY(0.0) + offset; p32.z = 0.0f;
  polygon.polygon.points.push_back(p32);
  p32.x = axis/2; p32.y = GetY(axis/2) + offset; p32.z = 0.0f;
  polygon.polygon.points.push_back(p32);
  p32.x = axis-d; p32.y = GetY(axis-d) + offset; p32.z = 0.0f;
  polygon.polygon.points.push_back(p32);
  p32.x = axis/2; p32.y = -GetY(axis/2) + offset; p32.z = 0.0f;
  polygon.polygon.points.push_back(p32);
  p32.x = 0.0f; p32.y = -GetY(0.0) + offset; p32.z = 0.0f;
  polygon.polygon.points.push_back(p32);
  p32.x = -axis/2; p32.y = -GetY(-axis/2) + offset; p32.z = 0.0f;
  polygon.polygon.points.push_back(p32);
  p32.x = -axis+d; p32.y = GetY(-axis+d) + offset; p32.z = 0.0f;
  polygon.polygon.points.push_back(p32);
  p32.x = -axis/2; p32.y = GetY(-axis/2) + offset; p32.z = 0.0f;
  polygon.polygon.points.push_back(p32);
}

void transform_polygon(const ef_point_t& check_pose,
  const geometry_msgs::msg::PolygonStamped& base_polygon,
  geometry_msgs::msg::PolygonStamped& check_polygon)
{
    auto Transform = [&](const ef_point_t& check_pose, const geometry_msgs::msg::Point32 point) {
        geometry_msgs::msg::Point32 new_point;
        new_point.x = check_pose.x + check_pose.cos_theta * point.x - check_pose.sin_theta * point.y;
        new_point.y = check_pose.y + check_pose.sin_theta * point.x + check_pose.cos_theta * point.y;
        return new_point;
    };

    int i = 0;
    check_polygon.polygon.points.resize(base_polygon.polygon.points.size());
    for (auto points : base_polygon.polygon.points) {
        check_polygon.polygon.points[i++] = Transform(check_pose, points);
    }
}

void set_cmdvel(const float linear_x, const float angular_z,
  geometry_msgs::msg::Twist& cmd_vel)
{
    cmd_vel.linear.x = linear_x;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.z = 0;
    cmd_vel.angular.z = 0;
    cmd_vel.angular.z = angular_z;
}

bool updateCheckPosesOnPath(
  const nav_msgs::msg::Path& path, std::vector<ef_point_t>& poses,
  const float step, const float len, const int start_idx)
{
  if (path.poses.empty()) {
    // ROS_ERROR("[EF ROS][%s,%d], path ahead empty !", __FUNCTION__, __LINE__);
    return false;
  }

  auto Pose2EFP = [&](const geometry_msgs::msg::PoseStamped& pose){
    ef_point_t p;
    p.x = pose.pose.position.x; p.y = pose.pose.position.y;
    p.theta = tf2::getYaw(pose.pose.orientation);
    p.cos_theta = cos(p.theta); p.sin_theta = sin(p.theta);
    return p;
  };

  poses.clear();
  poses.push_back(Pose2EFP(path.poses.at(start_idx)));
  float s = 0, l = 0, d;
  for (unsigned int i = start_idx+1, j = i-1; ; i ++, j = i-1) {
    if (i >= path.poses.size()) {
      i = 0;
      j = path.poses.size()-1;
    }
    d = std::hypot(path.poses.at(i).pose.position.x - path.poses.at(j).pose.position.x,
                   path.poses.at(i).pose.position.y - path.poses.at(j).pose.position.y);
    s += d;
    l += d;
    if (l > len) break;    
    if (s > step) {
      s = 0;
      poses.push_back(Pose2EFP(path.poses.at(i)));
    }
  }
  return true;
}

bool isCollisionOnPoses(std::vector<nav2_costmap_2d::MapLocation>& check_cells, 
  const std::vector<ef_point_t>& poses,
  const geometry_msgs::msg::PolygonStamped& polygon,
  nav2_costmap_2d::Costmap2D* map, const float sparse_size)
{
  bool is_collision = false;
  geometry_msgs::msg::PolygonStamped check_polygon;
  for (unsigned int i = 0; i < poses.size(); i ++) {
    transform_polygon(poses.at(i), polygon, check_polygon);
    is_collision = isCollisionInPolygon(check_cells, check_polygon, map, sparse_size);
    if (is_collision) {
      // ROS_DEBUG("[EF ROS][%s,%d], check poses [%d] collision", __FUNCTION__, __LINE__, i);
      break;
    }
  }

  return is_collision;
}

bool isCollisionInPolygon(std::vector<nav2_costmap_2d::MapLocation>& check_cells, 
    const geometry_msgs::msg::PolygonStamped& check_polygon,
    nav2_costmap_2d::Costmap2D* map, const float sparse_size,
    const unsigned char collision_value)
{
  std::vector<nav2_costmap_2d::MapLocation> map_polygon;
  nav2_costmap_2d::MapLocation map_point;
  for (auto point : check_polygon.polygon.points) {
    if (!map->worldToMap(point.x, point.y, map_point.x, map_point.y)) {
      // ROS_WARN("[EF ROS][%s,%d] Polygon point [%f,%f] lies outside map bounds !",
      //     __FUNCTION__, __LINE__, point.x, point.y);
      return true;
    }
    map_polygon.push_back(map_point);
  }

  if (sparse_size < 0) map->polygonOutlineCells(map_polygon, check_cells);
  else map->convexFillCellsSparsely(map_polygon, sparse_size, check_cells);
  
  for (auto cell : check_cells) {
    unsigned char cost = map->getCost(cell.x, cell.y);
    if (cost >= collision_value) return true;
  }

  return false;
}

float findClostest(const nav_msgs::msg::Path& path,
  const geometry_msgs::msg::PoseStamped& pose, int& index_min)
{
  if (path.poses.empty()) return -1.0;
  float dist_min = std::numeric_limits<float>::max();
  for (unsigned int i = 0; i < path.poses.size(); i ++) {
    float d = std::hypot(path.poses.at(i).pose.position.x - pose.pose.position.x,
                         path.poses.at(i).pose.position.y - pose.pose.position.y);
    if (d < dist_min) {
      dist_min = d;
      index_min = i;
    }
  }
  return dist_min;
}

} // namespace edge_follower_ns

