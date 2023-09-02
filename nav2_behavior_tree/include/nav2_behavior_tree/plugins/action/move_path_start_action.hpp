// Copyright (c) 2018 Intel Corporation
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__MOVE_PATH_START_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__MOVE_PATH_START_ACTION_HPP_

#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <string>

#include "nav_msgs/msg/path.h"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

class MovePathStart : public BT::ActionNodeBase
{
public:
  MovePathStart(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
        BT::OutputPort<nav_msgs::msg::Path>("path_out", "Path out"),
        BT::InputPort<nav_msgs::msg::Path>("path_in", "Path in"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>("pose", "Pose to draw path near"),
        BT::InputPort<double>("jump_dist", "Distance to jump")
    };
  }

private:
  void halt() override {}
  BT::NodeStatus tick() override;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__MOVE_PATH_START_ACTION_HPP_
