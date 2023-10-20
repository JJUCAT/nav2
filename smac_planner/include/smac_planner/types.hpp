// Copyright (c) 2020, Samsung Research America
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
// limitations under the License. Reserved.

#ifndef SMAC_PLANNER__TYPES_HPP_
#define SMAC_PLANNER__TYPES_HPP_

#include <vector>
#include <utility>

namespace smac_planner
{

typedef std::pair<float, unsigned int> NodeHeuristicPair;

/**
 * @struct smac_planner::SearchInfo
 * @brief Search properties and penalties
 */
struct SearchInfo
{
  float minimum_turning_radius;   // 最小转弯半径，SE2
  float non_straight_penalty;     // 非直线惩罚
  float change_penalty;           // 转向惩罚
  float reverse_penalty;          // 后退惩罚
  float cost_penalty;             // 代价(障碍物距离)惩罚
  float analytic_expansion_ratio; // TODO@LMR ???
};

}  // namespace smac_planner

#endif  // SMAC_PLANNER__TYPES_HPP_
