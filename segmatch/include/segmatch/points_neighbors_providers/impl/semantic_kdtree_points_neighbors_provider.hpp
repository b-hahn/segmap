#ifndef SEGMATCH_IMPL_SEMANTIC_KDTREE_POINTS_NEIGHBORS_PROVIDER_HPP_
#define SEGMATCH_IMPL_SEMANTIC_KDTREE_POINTS_NEIGHBORS_PROVIDER_HPP_

#include <glog/logging.h>

#include "segmatch/points_neighbors_providers/semantic_kdtree_points_neighbors_provider.hpp"

namespace segmatch {

// Force the compiler to reuse instantiations provided in kdtree_points_neighbors_provider.cpp
extern template class SemanticKdTreePointsNeighborsProvider<MapPoint>;

//=================================================================================================
//    SemanticKdTreePointsNeighborsProvider public methods implementation
//=================================================================================================

template<typename PointT>
void SemanticKdTreePointsNeighborsProvider<PointT>::update(
    const typename pcl::PointCloud<PointT>::ConstPtr point_cloud,
    const std::vector<int64_t>& points_mapping) {
  // Build k-d tree
  point_cloud_ = point_cloud;
  kd_tree_.setInputCloud(point_cloud_);
}

template<typename PointT>
const PointNeighbors SemanticKdTreePointsNeighborsProvider<PointT>::getNeighborsOf(
    const size_t point_index, const float search_radius) {
  CHECK(point_cloud_ != nullptr);

  // Get the neighbors from the kd-tree.
  std::vector<int> neighbors_indices;
  std::vector<float> neighbors_distances;
  kd_tree_.radiusSearch((*point_cloud_)[point_index], search_radius, neighbors_indices,
                        neighbors_distances);

  return neighbors_indices;
}

} // namespace segmatch

#endif // SEGMATCH_IMPL_SEMANTIC_KDTREE_POINTS_NEIGHBORS_PROVIDER_HPP_
