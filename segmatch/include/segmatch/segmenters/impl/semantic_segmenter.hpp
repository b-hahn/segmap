#ifndef SEGMATCH_IMPL_SEMANTIC_SEGMENTER_HPP_
#define SEGMATCH_IMPL_SEMANTIC_SEGMENTER_HPP_

#define PCL_NO_PRECOMPILE

#include "segmatch/segmenters/semantic_segmenter.hpp"

#include <laser_slam/benchmarker.hpp>

#include "segmatch/search/semantic_kdtree_flann.hpp"
#include "segmatch/segmented_cloud.hpp"

namespace segmatch {

// Force the compiler to reuse instantiations provided in euclidean_segmenter.cpp
extern template class SemanticSegmenter<MapPoint>;

//=================================================================================================
//    SemanticSegmenter public methods implementation
//=================================================================================================

template<typename ClusteredPointT>
SemanticSegmenter<ClusteredPointT>::SemanticSegmenter(
    const SegmenterParameters& params)
    : params_(params), min_segment_size_(params.min_cluster_size),
      max_segment_size_(params.max_cluster_size),
      radius_for_growing_(params.radius_for_growing) { }

template<typename ClusteredPointT>
void SemanticSegmenter<ClusteredPointT>::segment(
    const PointNormals& normals, const std::vector<bool>& is_point_modified, ClusteredCloud& cloud,
    PointsNeighborsProvider<MapPoint>& points_neighbors_provider, SegmentedCloud& segmented_cloud,
    std::vector<Id>& cluster_ids_to_segment_ids,
    std::vector<std::pair<Id, Id>>& renamed_segments) {
  BENCHMARK_BLOCK("SM.Worker.Segmenter");
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  // Clear segments.
  segmented_cloud.clear();

  //TODO(ben): make_shared() creates deep copy - figure otu some other way to this that avoid copying
  typename pcl::PointCloud <ClusteredPointT>::Ptr cloud_ptr = cloud.makeShared();
  // TODO(ben): or set tree to points_neighbors_provider.getPclSearchObject()
  typename pcl::search::Search<ClusteredPointT>::Ptr tree =
    boost::shared_ptr<pcl::search::Search<ClusteredPointT>>(new pcl::search::KdTree<ClusteredPointT>);
  // std::vector<pcl::PointIndices> input_cloud_indices;
  pcl::IndicesPtr indices (new std::vector<int>(cloud.size()));
  std::iota(indices->begin(), indices->end(), 0);
  
  std::vector<pcl::PointIndices> cluster_indices;

  // extract clusters using custom clustering
  //  - custom distance for hue
  //  - takes 4 or 5 dimensions
  //  - should I add region growing straight away as well? See how it's implemented. If it uses radiusSearch, then
  //    that's what I need to reimplement and I can reuse the rest of the code.

  // TODO: is the kdtree being constructed anywhere?
  std::cout << "==========================================================\n";
  std::cout << "num point in cloud:" << cloud.size() << std::endl;
  std::cout << "Constructing segmenter!" << std::endl;
  search::SemanticKdTreeFLANN<ClusteredPointT> segmenter(true);
  std::cout << "Setting input cloud!" << std::endl;
  segmenter.setInputCloud(cloud_ptr);
  std::cout << "Extracting clusters!" << std::endl;
  segmenter.extractEuclideanClusters(
      cloud, /* points_neighbors_provider.getPclSearchObject(), */ radius_for_growing_, cluster_indices,
      min_segment_size_,max_segment_size_);
  std::cout << "Done with extraction!" << std::endl;
  // segmenter.extractClusters(&point_indices);

  for (const auto& point_indices : cluster_indices) {
    segmented_cloud.addSegment(point_indices, cloud);
  }

  LOG(INFO) << "Segmentation complete. Found " << cluster_indices.size()
        << " clusters ."<< std::endl;
}

} // namespace segmatch

#endif // SEGMATCH_IMPL_SEMANTIC_SEGMENTER_HPP_
