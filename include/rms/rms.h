#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/scope_timer.h>

#include <rms/histogram.h>

namespace rms
{

/* //{ class RMS */

using t_points  = pcl::PointCloud<pcl::PointXYZ>::Ptr;
using t_voxel   = Eigen::Vector3i;

class RMS {

  // | ---------------------- Voxelization ---------------------- |
  struct VoxelHash
  {
    size_t operator()(const t_voxel& voxel) const {
      const std::uint32_t* vec = reinterpret_cast<const uint32_t*>(voxel.data());
      return ((1 << 20) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349663 ^ vec[2] * 83492791);
    }
  };

  // | ----------------------- Public API ----------------------- |
public:
  RMS(mrs_lib::ParamLoader& param_loader);
  void sample(sensor_msgs::PointCloud2::Ptr& msg_inout);

  // | --------------- ROS and conversion methods --------------- |
private:
  void fromROSMsg(const sensor_msgs::PointCloud2::Ptr cloud, t_points& pcl_cloud);
  void extractByIndices(sensor_msgs::PointCloud2::Ptr& msg_inout, t_indices& indices);

  // | ---------------- RMS variables and methods --------------- |
private:
  size_t _K            = 10;
  float  _lambda       = 1.0f;
  float  _voxel_input  = -1.0f;
  float  _voxel_output = -1.0f;

  void voxelizeIndices(const float voxel_size, const t_points pc_in, t_indices& indices_inout, const bool check_NaNs = true);

  std::vector<t_gfh> computeGFH(const t_points points, const t_indices& indices);
  t_indices          sampleByGFH(const t_points points, const std::vector<t_gfh>& gfh, const size_t K, const float lambda);
};

//}

}  // namespace rms
