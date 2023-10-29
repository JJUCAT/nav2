#include <edge_follower/nanoflann_port.h>

namespace nanoflann_port_ns
{
    
  NanoflannPort::NanoflannPort(const std::vector<geometry_msgs::msg::Point32>& pl)
  {
    Init(pl);
  }
    
  NanoflannPort::NanoflannPort(const nav_msgs::msg::Path& p)
  {
    Init(p);
  }

  NanoflannPort::~NanoflannPort()
  {
    Reset();
  }

  void NanoflannPort::Init(const std::vector<geometry_msgs::msg::Point32>& pl)
  {
    if (pl.empty()) return;

    Reset();

    pc_ = std::make_shared<PointCloud<float>>();
    pc_->pts.resize(pl.size());
    for (unsigned int i = 0; i < pl.size(); ++i) {
      pc_->pts.at(i).x = pl.at(i).x;
      pc_->pts.at(i).y = pl.at(i).y;
      pc_->pts.at(i).z = pl.at(i).z;
    }

    kdtree_ = std::make_shared<kd_tree_t>(3, *pc_, 10);
  }

  void NanoflannPort::Init(const nav_msgs::msg::Path& p)
  {
    if (p.poses.empty()) return;

    Reset();

    pc_ = std::make_shared<PointCloud<float>>();
    pc_->pts.resize(p.poses.size());
    for (unsigned int i = 0; i < p.poses.size(); ++i) {
      pc_->pts.at(i).x = p.poses.at(i).pose.position.x;
      pc_->pts.at(i).y = p.poses.at(i).pose.position.y;
      pc_->pts.at(i).z = p.poses.at(i).pose.position.z;
    }

    kdtree_ = std::make_shared<kd_tree_t>(3, *pc_, 10);
  }

  void NanoflannPort::Reset()
  {
    pc_.reset();
    kdtree_.reset();
  }

  size_t NanoflannPort::Size()
  {
    if (pc_ == nullptr) return 0;
    return pc_->pts.size();
  }

  geometry_msgs::msg::Point32 NanoflannPort::GetPoint(const int idx)
  {
    geometry_msgs::msg::Point32 p;
    if (pc_ == nullptr) {
      // ROS_ERROR("kd tree not initialized !");
      return p;
    }

    p.x = pc_->pts[idx].x;
    p.y = pc_->pts[idx].y;
    p.z = pc_->pts[idx].z;
    return p;
  }


  KDTIndex NanoflannPort::FindClosestPoint(const float x, const float y, const float z) const
  {
    float query_pnt[3] = {x, y, z};
    size_t num_results = 1;
    std::vector<unsigned int> ret_index(num_results);
    std::vector<float> out_dist_sqr(num_results);

    num_results = kdtree_->knnSearch(
        &query_pnt[0], num_results, &ret_index[0], &out_dist_sqr[0]);

    ret_index.resize(num_results);
    out_dist_sqr.resize(num_results);

    KDTIndex kdti;
    kdti.idx = ret_index[0];
    kdti.dist = out_dist_sqr[0];

    return kdti;
  }

  size_t NanoflannPort::FindClosestPoint(const float x, const float y, const float z,
    const size_t num, std::vector<KDTIndex>& idx_list) const
  {
    if (pc_ == nullptr) return 0;
    
    float query_pnt[3] = {x, y, z};
    size_t num_results = num;
    std::vector<unsigned int> ret_index(num_results);
    std::vector<float> out_dist_sqr(num_results);

    num_results = kdtree_->knnSearch(
        &query_pnt[0], num_results, &ret_index[0], &out_dist_sqr[0]);

    ret_index.resize(num_results);
    out_dist_sqr.resize(num_results);

    KDTIndex kdti;
    for (size_t i = 0; i < num_results; ++i) {
      kdti.idx = ret_index[0];
      kdti.dist = out_dist_sqr[0];
      idx_list.push_back(kdti);
    }

    return idx_list.size();
  }

  size_t NanoflannPort::FindPointsInRadius(const float x, const float y, const float z,
    const float r, std::vector<KDTIndex>& idx_list) const
  {
    if (pc_ == nullptr) return 0;

    float query_pnt[3] = {x, y, z};
    std::vector<std::pair<uint32_t, float>> ret_matches;
    size_t num = kdtree_->radiusSearch(&query_pnt[0], r, ret_matches, nanoflann::SearchParams());

    KDTIndex kdti;
    for (size_t i = 0; i < num; ++i) {
      kdti.idx = ret_matches.at(i).second;
      kdti.dist = ret_matches.at(i).first;
      idx_list.push_back(kdti);
    }

    return idx_list.size();
  }



} // namespace nanoflann_port_ns
