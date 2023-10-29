#ifndef NANOFLANN_PORT_H_
#define NANOFLANN_PORT_H_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nanoflann/nanoflann.hpp"
#include "nanoflann/utils.h"


namespace nanoflann_port_ns {

typedef struct
{
  unsigned int idx;
  float dist;
} KDTIndex;


class NanoflannPort
{
  using kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<float, PointCloud<float>>,
    PointCloud<float>, 3 /* dim */>;

 public:
  NanoflannPort() {}
  NanoflannPort(const std::vector<geometry_msgs::msg::Point32>& pl);
  NanoflannPort(const nav_msgs::msg::Path& p);
  ~NanoflannPort();

  void Init(const std::vector<geometry_msgs::msg::Point32>& pl);
  void Init(const nav_msgs::msg::Path& p);

  void Reset();

  size_t Size();

  geometry_msgs::msg::Point32 GetPoint(const int idx);

  KDTIndex FindClosestPoint(const float x, const float y, const float z) const;

  size_t FindClosestPoint(const float x, const float y, const float z,
    const size_t num, std::vector<KDTIndex>& idx_list) const;

  size_t FindPointsInRadius(const float x, const float y, const float z,
    const float r, std::vector<KDTIndex>& idx_list) const;

 private:
  std::shared_ptr<PointCloud<float>> pc_;
  std::shared_ptr<kd_tree_t> kdtree_;
};


} // namespace nanoflann_port_ns


#endif // NANOFLANN_PORT_H
