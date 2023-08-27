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

#include <chrono>
#include <string>

#include "nav2_behavior_tree/plugins/decorator/sequence_paths_export.hpp"

namespace nav2_behavior_tree
{

SequencePathsExport::SequencePathsExport(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
  getInput("path_list", path_list_);
}

BT::NodeStatus SequencePathsExport::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    current_path_index_ = 0;
    if (path_list_.empty()) {
      // RCLCPP_ERROR(node_->get_logger(), "empty path list !");
      std::cout << "empty path list !" << std::endl;
      std::stringstream error_msg;
      error_msg << "empty path list";
      throw std::runtime_error(error_msg.str());
    }

    setOutput("path", path_list_.at(current_path_index_));
    return BT::NodeStatus::RUNNING;
  }

  setStatus(BT::NodeStatus::RUNNING);
  const BT::NodeStatus child_state = child_node_->executeTick();
  switch (child_state) {
    case BT::NodeStatus::RUNNING:
      return BT::NodeStatus::RUNNING;

    case BT::NodeStatus::SUCCESS:
      if (current_path_index_++ >= path_list_.size()) {
        // RCLCPP_INFO(node_->get_logger(), "path list finish !");
        std::cout << "path list finish !" << std::endl;
        current_path_index_ = 0;
        return BT::NodeStatus::SUCCESS;
      } else {
        // RCLCPP_INFO(
        //   node_->get_logger(),
        //   "export path [%d], size:%u",
        //   current_path_index_, path_list_.at(current_path_index_).poses.size());
        std::cout << "export path [" << current_path_index_ << "], size:" <<
          path_list_.at(current_path_index_).poses.size() << std::endl;
        setOutput("path", path_list_.at(current_path_index_));
        return BT::NodeStatus::RUNNING;
      }

    case BT::NodeStatus::FAILURE:
        // RCLCPP_INFO(node_->get_logger(),
        //   "export path [%d] run failed",
        //   current_path_index_ = 0;
        std::cout << "export path [" << current_path_index_ << "] run failed" << std::endl;
        return BT::NodeStatus::FAILURE;
    default:
      return BT::NodeStatus::FAILURE;
  }
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::SequencePathsExport>("SequencePathsExport");
}
