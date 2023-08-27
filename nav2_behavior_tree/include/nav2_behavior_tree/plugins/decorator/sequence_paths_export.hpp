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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__SEQUENCE_PATHS_EXPORT_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__SEQUENCE_PATHS_EXPORT_HPP_

#include <chrono>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <string>

#include "behaviortree_cpp_v3/decorator_node.h"

namespace nav2_behavior_tree
{

class SequencePathsExport : public BT::DecoratorNode
{
public:
  SequencePathsExport(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  // Any BT node that accepts parameters must provide a requiredNodeParameters method
  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<std::vector<nav_msgs::msg::Path>>("path_list", "Paths to run"),
        BT::OutputPort<nav_msgs::msg::Path>("path", "current path to run"),
    };
  }

private:
  BT::NodeStatus tick() override;

  size_t current_path_index_ = 0;
  std::vector<nav_msgs::msg::Path> path_list_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__SEQUENCE_PATHS_EXPORT_HPP_
