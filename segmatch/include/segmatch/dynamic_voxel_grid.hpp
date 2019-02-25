#ifndef SEGMATCH_DYNAMIC_VOXEL_GRID_HPP_
#define SEGMATCH_DYNAMIC_VOXEL_GRID_HPP_

#include <algorithm>
#include <cmath>
#include <type_traits>
#include <vector>

#include <glog/logging.h>
#include <kindr/minimal/quat-transformation.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

namespace segmatch {

/// \brief A grid of cubic volume cells.
///
/// Points inserted in the grid are assigned to their respective voxels. The grid provides a
/// downsampled view of the points and supports removing of voxels according to a predicate.
/// The grid distinguishes between <em>active voxels</em> (voxels that contain a minimum number of
/// points) and \e inactive voxels.
/// \remark The class is \e not thread-safe. Concurrent access to the class results in undefined
/// behavior.
template<
  typename InputPointT,
  typename VoxelPointT,
  typename IndexT = uint64_t,
  uint8_t bits_x = 20,
  uint8_t bits_y = 20,
  uint8_t bits_z = 20>
class DynamicVoxelGrid {
 public:
  typedef typename pcl::PointCloud<InputPointT> InputCloud;
  typedef typename pcl::PointCloud<VoxelPointT> VoxelCloud;

  static_assert(std::is_integral<IndexT>::value && std::is_unsigned<IndexT>::value,
                "IndexT must be an unsigned integral type");
  static_assert(bits_x + bits_y + bits_z <= sizeof(IndexT) * 8,
                "The number of bits required per dimension is bigger than the size of IndexT");
  static_assert(bits_x > 0 && bits_y > 0 && bits_z > 0,
                "The index requires at least one bit per dimension");
  static_assert(pcl::traits::has_xyz<InputPointT>::value,
                "InputPointT must be a structure containing XYZ coordinates");
  static_assert(pcl::traits::has_xyz<VoxelPointT>::value,
                "VoxelPointT must be a structure containing XYZ coordinates");

  /// \brief Initializes a new instance of the DynamicVoxelGrid class.
  /// \param resolution Edge length of the voxels.
  /// \param min_points_per_voxel Minimum number of points that a voxel must contain in order to be
  /// considered \e active.
  /// \param origin The point around which the grid is centered.
  DynamicVoxelGrid(const float resolution, const int min_points_per_voxel,
                   const InputPointT& origin = InputPointT())
    : resolution_(resolution)
    , min_points_per_voxel_(min_points_per_voxel)
    , origin_(origin)
    , grid_size_(
        resolution * static_cast<float>(n_voxels_x),
        resolution * static_cast<float>(n_voxels_y),
        resolution * static_cast<float>(n_voxels_z))
    , origin_offset_(origin.getVector3fMap())
    , indexing_offset_(grid_size_ / 2.0f - origin_offset_)
    , world_to_grid_(1.0f / resolution)
    , min_corner_(origin_offset_ - grid_size_ / 2.0f)
    , max_corner_(origin_offset_ + grid_size_ / 2.0f)
    , active_centroids_(new VoxelCloud())
    , inactive_centroids_(new VoxelCloud())
    , pose_transformation_()
    , indexing_transformation_() {

    // Validate inputs.
    CHECK_GT(resolution, 0.0f);
    CHECK_GE(min_points_per_voxel, 1);
  }

  /// \brief Move constructor for the DynamicVoxelGrid class.
  /// \param other Object that has to be moved into the new instance.
  DynamicVoxelGrid(DynamicVoxelGrid&& other)
    : resolution_(other.resolution_)
    , min_points_per_voxel_(other.min_points_per_voxel_)
    , origin_(std::move(other.origin_))
    , grid_size_(std::move(other.grid_size_))
    , origin_offset_(std::move(other.origin_offset_))
    , indexing_offset_(std::move(other.indexing_offset_))
    , world_to_grid_(other.world_to_grid_)
    , min_corner_(std::move(other.min_corner_))
    , max_corner_(std::move(other.max_corner_))
    , active_centroids_(std::move(other.active_centroids_))
    , inactive_centroids_(std::move(other.inactive_centroids_))
    , voxels_(std::move(other.voxels_))
    , pose_transformation_(std::move(other.pose_transformation_))
    , indexing_transformation_(std::move(other.indexing_transformation_)) {
  }

  /// \brief Inserts a point cloud in the voxel grid.
  /// Inserting new points updates the X, Y, Z coordinates of the points, but leaves any extra
  /// fields untouched.
  /// \remark Insertion invalidates any reference to the centroids.
  /// \param new_cloud The new points that must be inserted in the grid.
  /// \returns Indices of the centroids of the voxels that have become \e active after the
  /// insertion.
  std::vector<int> insert(const InputCloud& new_cloud);

  /// \brief Result of a removal operation.
  /// \remark Enabled only for predicates of the form: <tt>bool p(const VoxelPointT&)</tt>
  template<typename Func>
  using RemovalResult = typename std::enable_if<
    std::is_convertible<Func, std::function<bool(const VoxelPointT&)>>::value,
    std::vector<bool>>::type;

  /// \brief Removes from the grid a set of voxels satisfying the given predicate.
  /// \remark Removal invalidates any reference to the centroids.
  /// \returns Vector indicating, for each active voxel index, if the centroid has been removed or
  /// not.
  template <typename Func>
  RemovalResult<Func> removeIf(Func predicate);

  /// \brief Compute the index of the voxel containing the specified point.
  template<typename PointXYZ_>
  IndexT getIndexOf(const PointXYZ_& point) const;

  /// \brief Apply a pose transformation to the voxel grid.
  /// \remark Multiple transformations are cumulative.
  /// \param transformation The transformation to be applied to the grid.
  void transform(const kindr::minimal::QuatTransformationTemplate<float>& transformation);

  /// \brief Clears the dynamic voxel grid, removing all the points it contains and resetting the
  /// transformations.
  void clear();

  /// \brief Returns a reference to the centroids of the active voxels.
  /// \remark Modifying the X, Y, Z components of the points in the returned cloud results in
  /// undefined behavior.
  /// \returns The centroids of the active voxels.
  inline VoxelCloud& getActiveCentroids() const { return *active_centroids_; }

  /// \brief Returns a reference to the centroids of the inactive voxels.
  /// \remark Modifying the X, Y, Z components of the points in the returned cloud results in
  /// undefined behavior.
  /// \returns The centroids of the inactive voxels.
  inline VoxelCloud& getInactiveCentroids() const { return *inactive_centroids_; }

  /// \brief Dump informations about the voxels contained in the grid.
  void dumpVoxels() const;

 private:
  // A point with its voxel index.
  struct IndexedPoint_ {
    IndexedPoint_(const InputPointT& point, const IndexT& voxel_index)
      : point(point), voxel_index(voxel_index) {
    }

    InputPointT point;
    IndexT voxel_index;
  };
  typedef std::vector<IndexedPoint_> IndexedPoints_;

  // A voxel in the grid.
  struct Voxel_ {
    Voxel_()
      : centroid(nullptr), index(0), num_points(0) {
        // TODO: change the size of the class counter to a config param depending on dataset used. This is currently set
        // to work with mapillary vistas.
        semantic_class_counter = std::vector<uint16_t>(64, 0);
        // std::cout << "Created Voxel with semantic_class_counter.size() = " << semantic_class_counter.size() << std::endl;
    }

    Voxel_(VoxelPointT* centroid, const IndexT& index, const uint32_t num_points, std::vector<uint16_t> semantic_class_counter)
      : centroid(centroid), index(index), num_points(num_points), semantic_class_counter(semantic_class_counter) {
      // semantic_class_counter = std::vector<uint8_t>(64, 0);
        // std::cout << "Created Voxel with semantic_class_counter.size() = " << semantic_class_counter.size() << std::endl;
    }

    Voxel_(const Voxel_& other)
      : centroid(other.centroid)
      , index(other.index)
      , num_points(other.num_points)
      , semantic_class_counter(other.semantic_class_counter)
    {
        // std::cout << "Created Voxel with semantic_class_counter.size() = " << semantic_class_counter.size()
        //           << std::endl;
    }

    VoxelPointT* centroid;
    // TODO: is uint16_t sufficiently large? Needs to be > than max number of points expected to be in a voxel
    std::vector<uint16_t> semantic_class_counter;
    IndexT index;
    uint32_t num_points;
  };

  // The data necessary to construct a voxel.
  struct VoxelData_ {
    Voxel_* old_voxel;
    typename IndexedPoints_::iterator points_begin;
    typename IndexedPoints_::iterator points_end;
  };

  // Compute the voxel indices of a point cloud and sort the points in increasing voxel index
  // order.
  IndexedPoints_ indexAndSortPoints_(const InputCloud& points) const;

  // Create a voxel staring from the data about the point it contains and insert it in the voxels
  // and centroids vectors. Returns true if the new points inserted triggered the voxel.
  bool createVoxel_(const IndexT index, const VoxelData_& data, std::vector<Voxel_>& new_voxels,
                    VoxelCloud& new_active_centroids, VoxelCloud& new_inactive_centroids);

  // Removes the centroids at the specified pointers. The pointers must be sorted in increasing
  // order.
  std::vector<bool> removeCentroids_(VoxelCloud& target_cloud, std::vector<VoxelPointT*> to_remove);

  std::vector<uint8_t>& getMostFrequentClass(std::vector<uint16_t> counter) {
    // find max element in old_voxel_->semantic_class_counter
    uint8_t class_id = distance(counter.begin(), std::max_element(counter.begin(), counter.end()));
    return class_id_to_color[class_id];
  }
  
  // Converts an rgb color to a mapillary class id
  std::map<std::vector<uint8_t>, uint8_t> color_to_class_id = {
      { { 165, 42, 42 }, 0 },    { { 0, 192, 0 }, 1 },      { { 196, 196, 196 }, 2 },  { { 190, 153, 153 }, 3 },
      { { 180, 165, 180 }, 4 },  { { 90, 120, 150 }, 5 },   { { 102, 102, 156 }, 6 },  { { 128, 64, 255 }, 7 },
      { { 140, 140, 200 }, 8 },  { { 170, 170, 170 }, 9 },  { { 250, 170, 160 }, 10 }, { { 96, 96, 96 }, 11 },
      { { 230, 150, 140 }, 12 }, { { 128, 64, 128 }, 13 },  { { 110, 110, 110 }, 14 }, { { 244, 35, 232 }, 15 },
      { { 150, 100, 100 }, 16 }, { { 70, 70, 70 }, 17 },    { { 150, 120, 90 }, 18 },  { { 220, 20, 60 }, 19 },
      { { 255, 0, 0 }, 20 },     { { 255, 0, 100 }, 21 },   { { 255, 0, 200 }, 22 },   { { 200, 128, 128 }, 23 },
      { { 255, 255, 255 }, 24 }, { { 64, 170, 64 }, 25 },   { { 230, 160, 50 }, 26 },  { { 70, 130, 180 }, 27 },
      { { 190, 255, 255 }, 28 }, { { 152, 251, 152 }, 29 }, { { 107, 142, 35 }, 30 },  { { 0, 170, 30 }, 31 },
      { { 255, 255, 128 }, 32 }, { { 250, 0, 30 }, 33 },    { { 100, 140, 180 }, 34 }, { { 220, 220, 220 }, 35 },
      { { 220, 128, 128 }, 36 }, { { 222, 40, 40 }, 37 },   { { 100, 170, 30 }, 38 },  { { 40, 40, 40 }, 39 },
      { { 33, 33, 33 }, 40 },    { { 100, 128, 160 }, 41 }, { { 142, 0, 0 }, 42 },     { { 70, 100, 150 }, 43 },
      { { 210, 170, 100 }, 44 }, { { 153, 153, 153 }, 45 }, { { 128, 128, 128 }, 46 }, { { 0, 0, 80 }, 47 },
      { { 250, 170, 30 }, 48 },  { { 192, 192, 192 }, 49 }, { { 220, 220, 0 }, 50 },   { { 140, 140, 20 }, 51 },
      { { 119, 11, 32 }, 52 },   { { 150, 0, 255 }, 53 },   { { 0, 60, 100 }, 54 },    { { 0, 0, 142 }, 55 },
      { { 0, 0, 90 }, 56 },      { { 0, 0, 230 }, 57 },     { { 0, 80, 100 }, 58 },    { { 128, 64, 64 }, 59 },
      { { 0, 0, 110 }, 60 },     { { 0, 0, 70 }, 61 },      { { 0, 0, 192 }, 62 },     { { 32, 32, 32 }, 63 },
      { { 120, 10, 10 }, 64 },   { { 0, 0, 0 }, 65 }
  };

  std::map<uint8_t, std::vector<uint8_t>> class_id_to_color = {
      { 0, { 165, 42, 42 } },    { 1, { 0, 192, 0 } },      { 2, { 196, 196, 196 } },  { 3, { 190, 153, 153 } },
      { 4, { 180, 165, 180 } },  { 5, { 90, 120, 150 } },   { 6, { 102, 102, 156 } },  { 7, { 128, 64, 255 } },
      { 8, { 140, 140, 200 } },  { 9, { 170, 170, 170 } },  { 10, { 250, 170, 160 } }, { 11, { 96, 96, 96 } },
      { 12, { 230, 150, 140 } }, { 13, { 128, 64, 128 } },  { 14, { 110, 110, 110 } }, { 15, { 244, 35, 232 } },
      { 16, { 150, 100, 100 } }, { 17, { 70, 70, 70 } },    { 18, { 150, 120, 90 } },  { 19, { 220, 20, 60 } },
      { 20, { 255, 0, 0 } },     { 21, { 255, 0, 100 } },   { 22, { 255, 0, 200 } },   { 23, { 200, 128, 128 } },
      { 24, { 255, 255, 255 } }, { 25, { 64, 170, 64 } },   { 26, { 230, 160, 50 } },  { 27, { 70, 130, 180 } },
      { 28, { 190, 255, 255 } }, { 29, { 152, 251, 152 } }, { 30, { 107, 142, 35 } },  { 31, { 0, 170, 30 } },
      { 32, { 255, 255, 128 } }, { 33, { 250, 0, 30 } },    { 34, { 100, 140, 180 } }, { 35, { 220, 220, 220 } },
      { 36, { 220, 128, 128 } }, { 37, { 222, 40, 40 } },   { 38, { 100, 170, 30 } },  { 39, { 40, 40, 40 } },
      { 40, { 33, 33, 33 } },    { 41, { 100, 128, 160 } }, { 42, { 142, 0, 0 } },     { 43, { 70, 100, 150 } },
      { 44, { 210, 170, 100 } }, { 45, { 153, 153, 153 } }, { 46, { 128, 128, 128 } }, { 47, { 0, 0, 80 } },
      { 48, { 250, 170, 30 } },  { 49, { 192, 192, 192 } }, { 50, { 220, 220, 0 } },   { 51, { 140, 140, 20 } },
      { 52, { 119, 11, 32 } },   { 53, { 150, 0, 255 } },   { 54, { 0, 60, 100 } },    { 55, { 0, 0, 142 } },
      { 56, { 0, 0, 90 } },      { 57, { 0, 0, 230 } },     { 58, { 0, 80, 100 } },    { 59, { 128, 64, 64 } },
      { 60, { 0, 0, 110 } },     { 61, { 0, 0, 70 } },      { 62, { 0, 0, 192 } },     { 63, { 32, 32, 32 } },
      { 64, { 120, 10, 10 } },   { 65, { 0, 0, 0 } }
  };

  // The centroids of the voxels containing enough points.
  std::unique_ptr<VoxelCloud> active_centroids_;
  std::unique_ptr<VoxelCloud> inactive_centroids_;

  // The voxels in the point cloud.
  std::vector<Voxel_> voxels_;

  // Properties of the grid.
  const float resolution_;
  const int min_points_per_voxel_;
  const InputPointT origin_;

  // Size of the voxel grid.
  static constexpr IndexT n_voxels_x = (IndexT(1) << bits_x);
  static constexpr IndexT n_voxels_y = (IndexT(1) << bits_y);
  static constexpr IndexT n_voxels_z = (IndexT(1) << bits_z);

  // Variables needed for conversion from world coordinates to voxel index.
  const Eigen::Vector3f grid_size_;
  const Eigen::Vector3f origin_offset_;
  const Eigen::Vector3f indexing_offset_;
  const Eigen::Vector3f min_corner_;
  const Eigen::Vector3f max_corner_;
  float world_to_grid_;
  kindr::minimal::QuatTransformationTemplate<float> pose_transformation_;
  kindr::minimal::QuatTransformationTemplate<float> indexing_transformation_;
}; // class DynamicVoxelGrid

// Short name macros for Dynamic Voxel Grid (DVG) template declaration and
// specification.
#define _DVG_TEMPLATE_DECL_ typename InputPointT, typename VoxelPointT, typename IndexT, uint8_t \
  bits_x, uint8_t bits_y, uint8_t bits_z
#define _DVG_TEMPLATE_SPEC_ InputPointT, VoxelPointT, IndexT, bits_x, bits_y, bits_z

//=================================================================================================
//    DynamicVoxelGrid public methods implementation
//=================================================================================================

template<_DVG_TEMPLATE_DECL_>
template <typename Func>
inline DynamicVoxelGrid<_DVG_TEMPLATE_SPEC_>::RemovalResult<Func>
DynamicVoxelGrid<_DVG_TEMPLATE_SPEC_>::removeIf(Func predicate) {
  // Setup iterators
  auto v_read = voxels_.begin();
  const auto v_end = voxels_.end();

  // Returns a reference to the point cloud containing the centroid of the
  // specified voxel.
  std::vector<VoxelPointT*> active_centroids_to_remove;
  std::vector<VoxelPointT*> inactive_centroids_to_remove;
  auto get_centroids_container_for = [&](const Voxel_& voxel)
      -> std::vector<VoxelPointT*>& {
    if (voxel.num_points >= min_points_per_voxel_) {
      return active_centroids_to_remove;
    } else {
      return inactive_centroids_to_remove;
    }
  };

  // Remove the voxels and collect the pointers of the centroids that must be
  // removed.
  while(v_read != v_end && !predicate(*(v_read->centroid)))
    ++v_read;

  if (v_read == v_end)
    return std::vector<bool>(active_centroids_->size(), false);
  auto v_write = v_read;
  get_centroids_container_for(*v_read).push_back(v_read->centroid);
  ++v_read;

  for(; v_read != v_end; ++v_read) {
    if(!predicate(*(v_read->centroid))) {
      // Copy the centroid, updating the pointer from the voxel.
      *v_write = *v_read;
      v_write->centroid -= get_centroids_container_for(*v_read).size();
      ++v_write;
    } else {
      // Keep track of the voxels that need to be deleted.
      get_centroids_container_for(*v_read).push_back(v_read->centroid);
    }
  }

  voxels_.erase(v_write, v_end);

  // Actually remove centroids
  removeCentroids_(*inactive_centroids_, inactive_centroids_to_remove);
  return removeCentroids_(*active_centroids_, active_centroids_to_remove);
}

} // namespace segmatch

#endif // SEGMATCH_DYNAMIC_VOXEL_GRID_HPP_
