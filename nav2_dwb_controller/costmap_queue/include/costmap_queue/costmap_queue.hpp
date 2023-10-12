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

#ifndef COSTMAP_QUEUE__COSTMAP_QUEUE_HPP_
#define COSTMAP_QUEUE__COSTMAP_QUEUE_HPP_

#include <cmath>
#include <vector>
#include <limits>
#include <memory>
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "costmap_queue/map_based_queue.hpp"

namespace costmap_queue
{
/**
 * @class CellData
 * @brief Storage for cell information used during queue expansion
 */
class CellData
{
public:
  /**
   * @brief Real Constructor
   * @param d The distance to the nearest obstacle
   * @param i The index of the cell in the costmap. Redundant with the following two parameters.
   * @param x The x coordinate of the cell in the cost map
   * @param y The y coordinate of the cell in the cost map
   * @param sx The x coordinate of the closest source cell in the costmap
   * @param sy The y coordinate of the closest source cell in the costmap
   */
  CellData(
    const double d, const unsigned int i, const unsigned int x, const unsigned int y,
    const unsigned int sx, const unsigned int sy)
  : distance_(d), index_(i), x_(x), y_(y), src_x_(sx), src_y_(sy)
  {
  }

  /**
   * @brief Default Constructor - Should be used sparingly
   */
  CellData()
  : distance_(std::numeric_limits<double>::max()), index_(0), x_(0), y_(0), src_x_(0), src_y_(0)
  {
  }

  static unsigned absolute_difference(const unsigned x, const unsigned y)
  {
    return (x > y) ? (x - y) : (y - x);
  }

  double distance_; // 地图栅格到最近障碍物距离
  unsigned int index_;
  unsigned int x_, y_;
  unsigned int src_x_, src_y_;
};

/**
 * @class CostmaQueue
 * @brief A tool for finding the cells closest to some set of originating cells.
 *
 * A common operation with costmaps is to define a set of cells in the costmap, and then
 * perform some operation on all the other cells based on which cell in the original set
 * the other cells are closest to. This operation is done in the inflation layer to figure out
 * how far each cell is from an obstacle, and is also used in a number of Trajectory cost functions.
 * 使用costmaps的一个常见操作是在costmap中定义一组单元格，
 * 然后根据原始集中其他单元格最接近的单元格对所有其他单元格执行一些操作
 * 这个操作是在膨胀层中完成的，用来计算每个单元离障碍物的距离，也用于许多轨迹成本函数。
 * It is implemented with a queue. The standard operation is to enqueueCell the original set, and then
 * retreive the other cells with the isEmpty/getNextCell iterator-like functionality. getNextCell
 * returns an object that contains the coordinates of this cell and the origin cell, as well as
 * the distance between them. By default, the Euclidean distance is used for ordering, but passing in
 * manhattan=true to the constructor will use the Manhattan distance.
 * 它是用队列实现的。标准操作是enqueueCell初始集合，然后使用类似于isEmpty/getNextCell迭代器的功能检索其他单元格
 * getNextCell返回一个对象，该对象包含该单元格和原始单元格的坐标，以及它们之间的距离
 * 默认情况下，欧几里得距离用于排序，但将manhattan=true传递给构造函数将使用曼哈顿距离。
 * The validCellToQueue overridable-function allows for deriving classes to limit the queue traversal
 * to a subset of all costmap cells. LimitedCostmapQueue does this by ignoring distances above a limit.
 * validCellToQueue 函数允许派生类将队列遍历限制为所有成本映射单元格的一个子集
 * LimitedCostmapQueue 通过忽略超过限制的距离来实现这一点。
 */
class CostmapQueue : public MapBasedQueue<CellData>
{
public:
  /**
   * @brief constructor
   * @param costmap Costmap which defines the size/number of cells
   * @param manhattan If true, sort cells by Manhattan distance, otherwise use Euclidean distance
   */
  explicit CostmapQueue(nav2_costmap_2d::Costmap2D & costmap, bool manhattan = false);

  /**
   * @brief Clear the queue
   */
  void reset() override;

  /**
   * @brief Add a cell the queue
   * @param x X coordinate of the cell
   * @param y Y coordinate of the cell
   */
  void enqueueCell(unsigned int x, unsigned int y);

  /**
   * @brief Get the next cell to examine, and enqueue its neighbors as needed
   * @return The next cell
   *
   * NB: Assumes that isEmpty has been called before this call and returned false
   */
  CellData getNextCell();

  /**
   * @brief Check to see if we should add this cell to the queue. Always true unless overridden.
   * @param cell The cell to check
   * @return True, unless overriden
   */
  virtual bool validCellToQueue(const CellData & /*cell*/) {return true;}
  /**
   * @brief convenience typedef for a pointer
   */
  typedef std::shared_ptr<CostmapQueue> Ptr;

protected:
  /**
   * @brief Enqueue a cell with the given coordinates and the given source cell
   */
  void enqueueCell(
    unsigned int index, unsigned int cur_x, unsigned int cur_y, unsigned int src_x,
    unsigned int src_y);

  /**
   * @brief Compute the cached distances
   */
  void computeCache();

  nav2_costmap_2d::Costmap2D & costmap_;
  std::vector<bool> seen_; // costmap 栅格访问记录
  int max_distance_;
  bool manhattan_; // 保存曼哈顿距离，即 x 轴 y 轴距离之和

protected:
  /**
   * @brief  Lookup pre-computed distances
   *        查表 (cur_x, cur_y) 和 (src_x, src_y) 的距离
   * @param cur_x The x coordinate of the current cell
   * @param cur_y The y coordinate of the current cell
   * @param src_x The x coordinate of the source cell
   * @param src_y The y coordinate of the source cell
   * @return
   */
  inline double distanceLookup(
    const unsigned int cur_x, const unsigned int cur_y,
    const unsigned int src_x, const unsigned int src_y)
  {
    unsigned int dx = CellData::absolute_difference(cur_x, src_x);
    unsigned int dy = CellData::absolute_difference(cur_y, src_y);
    return cached_distances_[dx][dy]; // 查表快速得到距离
  }
  std::vector<std::vector<double>> cached_distances_; // 记录 costmap 栅格距离最近障碍物距离
  int cached_max_distance_; // 上次 costmap 地图的最大边大小，单位栅格
};
}  // namespace costmap_queue

#endif  // COSTMAP_QUEUE__COSTMAP_QUEUE_HPP_
