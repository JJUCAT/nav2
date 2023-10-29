#ifndef BSPINE_PORT_H
#define BSPINE_PORT_H

#include <bspine/include/bspine.h>
#include <geometry_msgs/msg/point32.hpp>
#include <nav_msgs/msg/path.hpp>

namespace bspine_port_ns {

class BspinePort
{
 public:
  BspinePort() {}
  explicit BspinePort(const nav_msgs::msg::Path& path);
  ~BspinePort();

  bool twoOrderSmooth(const float sparse_interval, const float insert_interval,
    nav_msgs::msg::Path& smooth_path);

 private:
  bool GenData(const float sparse_interval, const float insert_interval, 
    std::vector<bspine_ns::CPosition>& cpoints, std::vector<int>& intnum);

  void UpdateOrientation(const bspine_ns::CPosition* cp,
    const int num, nav_msgs::msg::Path& smooth_path);

    
  nav_msgs::msg::Path path_;
};


} // namespace bspine_port_ns


#endif // BSPINE_PORT_H
