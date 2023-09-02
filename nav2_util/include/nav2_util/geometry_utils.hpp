// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_UTIL__GEOMETRY_UTILS_HPP_
#define NAV2_UTIL__GEOMETRY_UTILS_HPP_

#include <cmath>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/utils.h"

namespace nav2_util
{
namespace geometry_utils
{

inline geometry_msgs::msg::Quaternion orientationAroundZAxis(double angle)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, angle);  // void returning function
  return tf2::toMsg(q);
}

inline double euclidean_distance(
  const geometry_msgs::msg::Point & pos1,
  const geometry_msgs::msg::Point & pos2)
{
  double dx = pos1.x - pos2.x;
  double dy = pos1.y - pos2.y;
  double dz = pos1.z - pos2.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

inline double euclidean_distance(
  const geometry_msgs::msg::Pose & pos1,
  const geometry_msgs::msg::Pose & pos2)
{
  double dx = pos1.position.x - pos2.position.x;
  double dy = pos1.position.y - pos2.position.y;
  double dz = pos1.position.z - pos2.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

inline double euclidean_distance(
  const geometry_msgs::msg::PoseStamped & pos1,
  const geometry_msgs::msg::PoseStamped & pos2)
{
  return euclidean_distance(pos1.pose, pos2.pose);
}

inline bool in_polygon(
  const geometry_msgs::msg::Polygon& polygon,
  const geometry_msgs::msg::Point p)
{
  bool in = false;
  for (unsigned int i = 0, j = polygon.points.size()-1;
    i < polygon.points.size(); j = i, i++) {
    geometry_msgs::msg::Point32 start =
      polygon.points.at(i), end = polygon.points.at(j);
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

inline double get_diff_angle(const double angle_start, const double angle_end)
{
  double diff_angle = angle_end - angle_start;
  if (diff_angle > M_PI) {
      diff_angle = diff_angle - 2 * M_PI;
  } else if (diff_angle < -M_PI) {
      diff_angle = 2 * M_PI + diff_angle;
  }
  return diff_angle;
}

inline void transform_polygon(const geometry_msgs::msg::PoseStamped& check_pose,
  const geometry_msgs::msg::PolygonStamped& base_polygon,
  geometry_msgs::msg::PolygonStamped& check_polygon)
{
  double yaw = tf2::getYaw(check_pose.pose.orientation);
  double cos_theta = cos(yaw), sin_theta = sin(yaw);
  for (auto point : base_polygon.polygon.points) {
    geometry_msgs::msg::Point32 new_point;
    new_point.x = check_pose.pose.position.x + cos_theta * point.x - sin_theta * point.y;
    new_point.y = check_pose.pose.position.y + sin_theta * point.x + cos_theta * point.y;
    check_polygon.polygon.points.push_back(new_point);
  }
}


}  // namespace geometry_utils
}  // namespace nav2_util

#endif  // NAV2_UTIL__GEOMETRY_UTILS_HPP_
