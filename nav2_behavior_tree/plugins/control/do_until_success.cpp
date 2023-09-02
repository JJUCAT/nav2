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

#include <string>
#include "nav2_behavior_tree/plugins/control/do_until_success.hpp"

namespace nav2_behavior_tree
{

DoUntilSuccess::DoUntilSuccess(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ControlNode::ControlNode(name, conf)
{
}

BT::NodeStatus DoUntilSuccess::tick()
{
  const unsigned children_count = children_nodes_.size();

  if (children_count != 2) {
    throw BT::BehaviorTreeException("DoUntilSuccess must only have 2 children.");
  }

  setStatus(BT::NodeStatus::RUNNING);
  std::cout << "[DUS] check finish" << std::endl;
  TreeNode * child_node = children_nodes_[0];
  if (!init_ || woke_finish_) {
    init_ = true;
    const BT::NodeStatus child_status = child_node->executeTick();
    std::cout << "[DUS] child status: " << int(child_status) << std::endl;
    if (child_status == BT::NodeStatus::SUCCESS && woke_finish_) {
      halt();
      init_ = false;
      woke_finish_ = false;
      return BT::NodeStatus::SUCCESS;
    } else if (child_status == BT::NodeStatus::FAILURE) {
      halt();
      init_ = false;
      woke_finish_ = false;
      return BT::NodeStatus::FAILURE;
    }
  }
  std::cout << "[DUS] work" << std::endl;
  child_node = children_nodes_[1];
  const BT::NodeStatus child_status = child_node->executeTick();
  std::cout << "[DUS] child status: " << int(child_status) << std::endl;
  if (child_status == BT::NodeStatus::SUCCESS) {
    woke_finish_ = true;
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::RUNNING;
  }
}

void DoUntilSuccess::halt()
{
  ControlNode::halt();
  woke_finish_ = true;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::DoUntilSuccess>("DoUntilSuccess");
}
