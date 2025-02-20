#include "rms/rms.h"
#include <tsl/robin_map.h>  // Include here since it's a header-only library

namespace rms
{

/*//{ RMS() constructor */
RMS::RMS(mrs_lib::ParamLoader &param_loader) {
  _K = size_t(param_loader.loadParamReusable2<int>("K"));
  param_loader.loadParam<float>("lambda", _lambda);
  param_loader.loadParam<float>("voxelization/input", _voxel_input);
  param_loader.loadParam<float>("voxelization/output", _voxel_output);
}
/*//}*/

/*//{ sample() */
void RMS::sample(sensor_msgs::PointCloud2::Ptr &msg_inout) {

  // ROS -> PCL conversion
  t_points pts;
  this->fromROSMsg(msg_inout, pts);

  if (pts->empty()) {
    ROS_WARN("[RMS] Empty cloud received. Nothing to sample.");
    return;
  }

  // Setup a vector of point indices 0->N
  t_indices indices = t_indices(pts->size());
  std::iota(std::begin(indices), std::end(indices), 0);

  // L11: pre-voxelize
  voxelizeIndices(_voxel_input, pts, indices, true);

  // L12-L14: compute GFH
  const auto &gfh = computeGFH(pts, indices);

  // L15-L28: GFH entropy minimization
  indices = sampleByGFH(pts, gfh, size_t(_K), _lambda);

  // post-voxelize if needed
  if (_voxel_output > _voxel_input) {
    voxelizeIndices(_voxel_output, pts, indices, false);
  }

  // L29: sample the input message by indices
  extractByIndices(msg_inout, indices);
}
/*//}*/

/*//{ sampleByGFH() */
t_indices RMS::sampleByGFH(const t_points points, const std::vector<t_gfh> &gfh, const size_t K, const float lambda) {

  // Construct histogram of the 1D values
  auto hist = Histogram1D(K, 0.0f, 1.0f, gfh);

  size_t N        = 0;
  float  max_rate = 0.0;

  while (N < points->size()) {

    // Sample one point
    float entropy;
    hist.selectByUniformnessMaximization(entropy);
    N++;

    // Eq. (28): compute mean entropy
    const float rate = entropy / N;

    // L20-L23: Add first K samples to initialize
    if (N <= K) {

      // Eq. (31): Compute maximum entropy rate
      max_rate = std::fmax(rate, max_rate);

    } else {

      // L26: Break if rate of entropy change has slowed down under a threshold
      if ((rate / max_rate) < lambda) {
        break;
      }
    }
  }

  // Retrieve and return the selected indices and return them
  return hist.getSelectedIndices();
}
/*//}*/

/*//{ computeGFH() */
std::vector<t_gfh> RMS::computeGFH(const t_points points_in, const t_indices &indices_in) {

  // Setup output vector
  std::vector<t_gfh> gfh;
  gfh.reserve(indices_in.size());

  // L12: Construct KDTree
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

  // Need to convert our index vector to PCL indices format
  const pcl::IndicesPtr pcl_indices = boost::make_shared<pcl::Indices>();
  pcl_indices->reserve(indices_in.size());
  for (const auto idx : indices_in) {
    pcl_indices->push_back(idx);
  }

  kdtree.setInputCloud(points_in, pcl_indices);

  float       max_gfh_norm = 0.0f;                // <0, 1> normalizing factor for GFH norm
  const float nn_radius    = 2.0 * _voxel_input;  // Nearest-neighbor search radius
  /* const float nn_radius    = 2.236f * _voxel_input;  // Nearest-neighbor search radius (linear scale: sqrt(5)) */

  for (const auto pt_idx : indices_in) {

    // Get point at index
    const auto &pt = points_in->at(pt_idx);

    // Perform radius search
    std::vector<int>   radius_indices;
    std::vector<float> radius_sq_dist;
    /* kdtree.nearestKSearch(pt, 5, radius_indices, radius_sq_dist); // optional search */
    kdtree.radiusSearch(pt, nn_radius, radius_indices, radius_sq_dist);

    // Compute GFH vector
    size_t          N      = 0;
    Eigen::Vector3f pt_gfh = Eigen::Vector3f::Zero();
    for (size_t i = 0; i < radius_indices.size(); i++) {

      const auto ind = radius_indices[i];

      if (ind == pt_idx) {
        continue;
      }

      const auto &pt_neigh = points_in->at(ind);
      pt_gfh += Eigen::Vector3f(pt_neigh.x - pt.x, pt_neigh.y - pt.y, pt_neigh.z - pt.z);
      N++;
    }

    if (N > 1) {
      pt_gfh /= float(N);
    }

    // Compute the GFH norm
    const float pt_gfh_norm = pt_gfh.norm();
    if (pt_gfh_norm > max_gfh_norm) {
      max_gfh_norm = pt_gfh_norm;
    }

    gfh.emplace_back(pt_idx, pt_gfh_norm);
  }

  // Normalize to <0, 1>
  if (max_gfh_norm > 0.0f) {
    std::for_each(gfh.begin(), gfh.end(), [&max_gfh_norm](auto &gfh) { gfh.second /= max_gfh_norm; });
  }

  return gfh;
}
/*//}*/

/*//{ extractByIndices() */
void RMS::extractByIndices(sensor_msgs::PointCloud2::Ptr &msg_inout, t_indices &indices) {

  const auto point_step = msg_inout->point_step;

  msg_inout->is_dense = true;
  msg_inout->width    = indices.size();
  msg_inout->height   = 1;
  msg_inout->row_step = msg_inout->width * point_step;

  // prepare sampled data (high compression is expected -> creating new data contained is faster then removing points from msg_inout->data)
  std::vector<std::uint8_t> data_out;
  data_out.reserve(indices.size() * point_step);

  const auto data_in_begin = msg_inout->data.begin();
  for (const auto i : indices) {
    const size_t pt_idx = i * point_step;
    std::copy(data_in_begin + pt_idx, data_in_begin + pt_idx + point_step, std::back_inserter(data_out));
  }

  msg_inout->data = std::move(data_out);
}
/*//}*/

/*//{ voxelizeIndices() */
void RMS::voxelizeIndices(const float voxel_size, const t_points pc_in, t_indices &indices_inout, const bool check_NaNs) {

  // initialize hash set (key: voxel, value: index in pc_in, hashing: VoxelHash)
  tsl::robin_map<t_voxel, size_t, VoxelHash> grid;
  grid.reserve(indices_inout.size());

  // voxelize: keep first point inserted into the voxel
  // other metric for sampling can be used if needed (e.g., the point closest to the voxel center)
  for (const auto idx : indices_inout) {
    const auto &pt = pc_in->at(idx);

    // remove NaNs
    if (check_NaNs && (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))) {
      continue;
    }

    const auto pt_eig = Eigen::Vector3f(pt.x, pt.y, pt.z);
    const auto voxel = t_voxel((pt_eig / voxel_size).cast<int>());

    // voxel occupied: erase this point's index
    if (grid.contains(voxel)) {
      continue;
    }

    // voxel unoccupied: keep this point's index and insert it to the voxel hashset
    grid.insert({voxel, idx});
  }

  // Store voxelized indices
  size_t i = 0;
  for (const auto &key : grid) {
    indices_inout.at(i++) = key.second;
  }
  indices_inout.resize(grid.size());
}
/*//}*/

/*//{ fromROSMsg() */
/* Faster pcl::fromROSMsg as taken from: https://gist.github.com/facontidavide/237e64dd5c8c697c97f8c75c14e73f7c */
void RMS::fromROSMsg(const sensor_msgs::PointCloud2::Ptr cloud, t_points &pcl_cloud) {
  pcl_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  // Copy info fields
  pcl_conversions::toPCL(cloud->header, pcl_cloud->header);
  pcl_cloud->width    = cloud->width;
  pcl_cloud->height   = cloud->height;
  pcl_cloud->is_dense = cloud->is_dense == 1;

  pcl::MsgFieldMap                field_map;
  std::vector<pcl::PCLPointField> msg_fields;
  pcl_conversions::toPCL(cloud->fields, msg_fields);
  pcl::createMapping<pcl::PointXYZ>(msg_fields, field_map);

  // Copy point data
  std::uint32_t num_points = cloud->width * cloud->height;
  pcl_cloud->points.resize(num_points);
  std::uint8_t *cloud_data = reinterpret_cast<std::uint8_t *>(&pcl_cloud->points[0]);

  // Check if we can copy adjacent points in a single memcpy.  We can do so if there
  // is exactly one field to copy and it is the same size as the source and destination
  // point types.
  if (field_map.size() == 1 && field_map[0].serialized_offset == 0 && field_map[0].struct_offset == 0 && field_map[0].size == cloud->point_step &&
      field_map[0].size == sizeof(pcl::PointXYZ)) {
    std::uint32_t       cloud_row_step = static_cast<std::uint32_t>(sizeof(pcl::PointXYZ) * pcl_cloud->width);
    const std::uint8_t *msg_data       = &cloud->data[0];
    // Should usually be able to copy all rows at once
    if (cloud->row_step == cloud_row_step) {
      memcpy(cloud_data, msg_data, cloud->data.size());
    } else {
      for (std::uint32_t i = 0; i < cloud->height; ++i, cloud_data += cloud_row_step, msg_data += cloud->row_step)
        memcpy(cloud_data, msg_data, cloud_row_step);
    }
  } else {
    // If not, memcpy each group of contiguous fields separately
    for (std::uint32_t row = 0; row < cloud->height; ++row) {
      const std::uint8_t *row_data = &cloud->data[row * cloud->row_step];
      for (std::uint32_t col = 0; col < cloud->width; ++col) {
        const std::uint8_t *msg_data = row_data + col * cloud->point_step;
        for (const pcl::detail::FieldMapping &mapping : field_map) {
          memcpy(cloud_data + mapping.struct_offset, msg_data + mapping.serialized_offset, mapping.size);
        }
        cloud_data += sizeof(pcl::PointXYZ);
      }
    }
  }
}
/*//}*/

}  // namespace rms
