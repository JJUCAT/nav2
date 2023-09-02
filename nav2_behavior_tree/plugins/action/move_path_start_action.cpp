// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Francisco Martin Rico
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

#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <string>
#include <memory>
#include <limits>

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"

#include "nav2_behavior_tree/plugins/action/move_path_start_action.hpp"

namespace nav2_behavior_tree
{

MovePathStart::MovePathStart(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
}

inline BT::NodeStatus MovePathStart::tick()
{
  nav_msgs::msg::Path path_in, path_out;
  geometry_msgs::msg::PoseStamped pose;
  double jump_dist;
  getInput("path_in", path_in);
  getInput("pose", pose);
  getInput("jump_dist", jump_dist);


  auto logger = config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger();
  if (path_in.poses.empty()) {
    std::cout << "input path empty !" << std::endl;
    RCLCPP_ERROR(logger, "Input path is empty.");
    setOutput("path_out", path_out);
    return BT::NodeStatus::FAILURE;
  }
  
  double dist, dist_min = std::numeric_limits<double>::max();
  unsigned int i_min = 0;
  for (unsigned int i = 0; i < path_in.poses.size(); ++i) {
    dist = nav2_util::geometry_utils::euclidean_distance(
             pose, path_in.poses.at(i));
    if (dist < dist_min) {
      dist_min = dist;
      i_min = i;
    }
  }
  RCLCPP_INFO(logger,
    "Path size is %u, clostest index is %u", path_in.poses.size(), i_min);

  double dist_sum = 0.0;
  unsigned jump_index = i_min;
  if (jump_index > 0) {
    for (unsigned int start = i_min, end = start+1; ; start ++, end = start+1) {
      if (start >= path_in.poses.size()) {
        start = 0; end = 1;
      } else if (end >= path_in.poses.size()) {
        end = 0;
      }
      dist_sum += nav2_util::geometry_utils::euclidean_distance(
              path_in.poses.at(start), path_in.poses.at(end));
      if (dist_sum >= jump_dist) {
        jump_index = end;
        break;
      };
    }
    RCLCPP_INFO(logger,
      "After jump , index is %u", jump_index);
  }
  
  path_out.header = path_in.header;
  path_out.poses.assign(path_in.poses.begin() + jump_index, path_in.poses.end());
  if (jump_index > 0) {
    path_out.poses.insert(
      path_out.poses.end(), path_in.poses.begin(), path_in.poses.begin() + jump_index);    
  }
  RCLCPP_INFO(logger, "path_out size: %u", path_out.poses.size());
  setOutput("path_out", path_out);
  
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::MovePathStart>("MovePathStart");
}
