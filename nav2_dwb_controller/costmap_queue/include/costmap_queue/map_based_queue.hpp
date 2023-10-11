/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef COSTMAP_QUEUE__MAP_BASED_QUEUE_HPP_
#define COSTMAP_QUEUE__MAP_BASED_QUEUE_HPP_

#include <algorithm>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

namespace costmap_queue
{
/**
 * @brief Templatized interface for a priority queue
 *
 * This is faster than the std::priority_queue implementation in certain cases because iterating does
 * not require resorting after every element is examined.
 * Based on https://github.com/ros-planning/navigation/pull/525
 * The relative speed of this against the priority queue depends how many items with each
 * priority are inserted into the queue.
 *
 * One additional speed up depends on the patterns of priorities during each iteration of the queue.
 * If the same priorities are inserted into the queue on every iteration, then it is quicker to
 * set reset_bins = false, such that the priority bins are not reset and will not have to be recreated
 * on each iteration.
 */

// 素材箱子，装有带着各种优先级的素材组
template<class item_t>
class MapBasedQueue
{
public:
  /**
   * @brief Default Constructor
   */
  explicit MapBasedQueue(bool reset_bins = true)
  : reset_bins_(reset_bins), item_count_(0)
  {
    reset();
  }

  /**
   * @brief Clear the queue
   */
  virtual void reset()
  {
    if (reset_bins_ || item_count_ > 0) {
      item_bins_.clear();
      item_count_ = 0;
    }
    iter_ = last_insert_iter_ = item_bins_.end();
  }

  /**
   * @brief Add a new item to the queue with a set priority，添加素材
   * @param priority Priority of the item，优先级
   * @param item Payload item，单个素材
   */
  void enqueue(const double priority, item_t item)
  {
    // We keep track of the last priority we inserted. If this items priority
    // matches the previous insertion we can avoid searching through all the
    // bins.
    // 空素材，或者上个素材的优先级不同
    if (last_insert_iter_ == item_bins_.end() || last_insert_iter_->first != priority) {
      last_insert_iter_ = item_bins_.find(priority);

      // If not found, create a new bin，是一个新的优先级，创建新的优先级素材组
      if (last_insert_iter_ == item_bins_.end()) {
        auto map_item = std::make_pair(priority, std::move(std::vector<item_t>()));

        // Inserts an item if it doesn't exist. Returns an iterator to the item
        // whether it existed or was inserted.
        std::pair<ItemMapIterator, bool> insert_result = item_bins_.insert(std::move(map_item));
        last_insert_iter_ = insert_result.first;
      }
    }

    // Add the item to the vector for this map key
    last_insert_iter_->second.push_back(item);
    item_count_++;

    // Use short circuiting to check if we want to update the iterator
    // iter_ 指向低优先级的素材组
    if (iter_ == item_bins_.end() || priority < iter_->first) {
      iter_ = last_insert_iter_;
    }
  }

  /**
   * @brief Check to see if there is anything in the queue
   * @return True if there is nothing in the queue
   *
   * Must be called prior to front/pop.
   */
  bool isEmpty()
  {
    return item_count_ == 0;
  }

  /**
   * @brief Return the item at the front of the queue
   * 返回当前优先级下的素材组最后一个素材，是按优先级从低到高检查的
   * @return The item at the front of the queue
   */
  item_t & front()
  {
    if (iter_ == item_bins_.end()) {
      throw std::out_of_range("front() called on empty costmap_queue::MapBasedQueue!");
    }

    return iter_->second.back();
  }

  /**
   * @brief Remove (and destroy) the item at the front of the queue
   */
  void pop()
  {
    if (iter_ != item_bins_.end() && !iter_->second.empty()) {
      iter_->second.pop_back();
      item_count_--;
    }
    // map 是有序容器，默认按 key 从小到大排序，find_if 是有序遍历 map 的
    // 上面弹出一个素材后，如果当前的优先级素材空了，iter_ 会指向下一个优先级素材组
    auto not_empty = [](const typename ItemMap::value_type & key_val) {
        return !key_val.second.empty();
      };
    iter_ = std::find_if(iter_, item_bins_.end(), not_empty);
  }

protected:
  // map 容器，包含多个“带优先级的一组素材”，称为素材箱子
  using ItemMap = std::map<double, std::vector<item_t>>; 
  using ItemMapIterator = typename ItemMap::iterator;

  bool reset_bins_;

  ItemMap item_bins_; // 素材箱子
  unsigned int item_count_; // 不同优先级素材的总数量
  ItemMapIterator iter_; // 因为 map 是默认 key 从小到大排序的，所以 iter_ 默认指向低优先级素材组
  ItemMapIterator last_insert_iter_;
};
}  // namespace costmap_queue

#endif  // COSTMAP_QUEUE__MAP_BASED_QUEUE_HPP_
