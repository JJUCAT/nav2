#include <edge_follower/bspine_port.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace bspine_port_ns
{

  BspinePort::BspinePort(const nav_msgs::msg::Path& path)
  {
    path_ = path;
  }

  BspinePort::~BspinePort()
  {

  }

  bool BspinePort::twoOrderSmooth(const float sparse_interval, const float insert_interval,
    nav_msgs::msg::Path& smooth_path)
  {
    std::vector<bspine_ns::CPosition> cpoints;
    std::vector<int> intnum;
    if (GenData(sparse_interval, insert_interval, cpoints, intnum)) {
      bspine_ns::CPosition* testpt = new bspine_ns::CPosition[cpoints.size()];
      for(unsigned int i=0; i<cpoints.size(); i++) {
        testpt[i] = bspine_ns::CPosition(cpoints.at(i).x, cpoints.at(i).y);
      }
      int* insert_num = new int[intnum.size()];
      for(unsigned int i=0; i<intnum.size(); i++) {
        insert_num[i] = intnum.at(i);
      }
      bspine_ns::CBSpline bspline;
      int num = cpoints.size();
      bspline.TwoOrderBSplineInterpolatePt(testpt, num, insert_num);
      // delete testpt; // testpt 在 TwoOrderBSplineInterpolatePt 内已经清除内存了
      delete insert_num;
      
      if (num < 2) {
        // ROS_ERROR("[EF ROS][%s,%d], smooth failed !", __FUNCTION__, __LINE__);
        return false;
      }
      UpdateOrientation(testpt, num, smooth_path);
      return true;
    }
    // ROS_ERROR("[EF ROS][%s,%d], generate data failed !", __FUNCTION__, __LINE__);
    return false;
  }

// -------------------- private methods --------------------

bool BspinePort::GenData(const float sparse_interval, const float insert_interval, 
  std::vector<bspine_ns::CPosition>& cpoints, std::vector<int>& intnum)
{
  if (path_.poses.size() <= 2) {
    // ROS_WARN("[EF ROS][%s,%d], less poses to smooth .", __FUNCTION__, __LINE__);
    return false;
  }

  cpoints.push_back(bspine_ns::CPosition(
    path_.poses.front().pose.position.x, path_.poses.front().pose.position.y));
  float dist_sum = 0;
  for (unsigned int i = 0; i < path_.poses.size()-1; i ++) {
    dist_sum += std::hypot(path_.poses.at(i).pose.position.x - path_.poses.at(i+1).pose.position.x,
                            path_.poses.at(i).pose.position.y - path_.poses.at(i+1).pose.position.y);
    if (dist_sum >= sparse_interval) {
      dist_sum = 0;
      cpoints.push_back(bspine_ns::CPosition(
            path_.poses.at(i+1).pose.position.x, path_.poses.at(i+1).pose.position.y));
    }
  }
  if (dist_sum > insert_interval*1.5+0.01) {
    cpoints.push_back(bspine_ns::CPosition(
      path_.poses.back().pose.position.x, path_.poses.back().pose.position.y));
  }

  int insert_num = static_cast<int>(sparse_interval/insert_interval-0.5);
  if (insert_num <= 0) {
    // ROS_WARN("[EF ROS][%s,%d], sparse point too close .", __FUNCTION__, __LINE__);
    return false;
  }
  intnum.resize(cpoints.size()-1, insert_num);
  intnum.back() = static_cast<int>(dist_sum/insert_interval-0.5);
  return true;
}

void BspinePort::UpdateOrientation(const bspine_ns::CPosition* cp,
  const int num, nav_msgs::msg::Path& smooth_path)
{
  smooth_path.header = path_.header;
  smooth_path.poses.clear();
  for (int i = 0; i < num-1; i ++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = smooth_path.header;
    pose.pose.position.x = cp[i].x;
    pose.pose.position.y = cp[i].y;
    float yaw = atan2(cp[i+1].y - cp[i].y, cp[i+1].x - cp[i].x);
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);    
    pose.pose.orientation = tf2::toMsg(q);
    smooth_path.poses.push_back(pose);
  }
  geometry_msgs::msg::PoseStamped pose;
  pose.header = smooth_path.header;
  pose.pose.position.x = cp[num-1].x;
  pose.pose.position.y = cp[num-1].y;
  pose.pose.orientation = smooth_path.poses.back().pose.orientation;
  smooth_path.poses.push_back(pose);
}

} // namespace bspine_port_ns



