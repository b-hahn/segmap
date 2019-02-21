#ifndef SEGMATCH_POINT_COLOR_SEMANTICS_HPP_
#define SEGMATCH_POINT_COLOR_SEMANTICS_HPP_

#include <stdint.h>

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/impl/instantiate.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/features/esf.h>
#include <pcl/features/impl/esf.hpp>
#include <pcl/pcl_base.h>
#include <pcl/impl/pcl_base.hpp>



/// \brief Custom PCL point type that can be extended with custom fields needed by incremental
/// algorithms. Custom points must be defined and registered in the global namespace.
struct _SegMatch_PointColorSemantics : public pcl::PointXYZRGB {
  using pcl::PointXYZRGB::data;

  void print_data() {
    std::cout <<  data[0];
  }


  inline _SegMatch_PointColorSemantics(const _SegMatch_PointColorSemantics &p) {
    data[0] = p.x;
    data[1] = p.y;
    data[2] = p.z;
    data[3] = 1.0f;
    ed_cluster_id = p.ed_cluster_id;
    sc_cluster_id = p.sc_cluster_id;
    rgb = p.rgb;
    semantics_rgb = p.semantics_rgb;
  }

  inline _SegMatch_PointColorSemantics()
  {
    data[0] = 0.0f;
    data[1] = 0.0f;
    data[2] = 0.0f;
    data[3] = 1.0f;
    ed_cluster_id = 0u;
    sc_cluster_id = 0u;
    rgb = 15329511.0f;  // TODO(ben): current equal to (233, 232, 231) for testing
    semantics_rgb = 15329511;
  }

  // TODO: not sure if this is the right constructor. Should replace the PointXYZRGB(int r, int g, int b) constructor but make sure 1) semantics aren't needed here yet
  inline _SegMatch_PointColorSemantics(int r, int g, int b)
  {
    data[0] = 0.0f;
    data[1] = 0.0f;
    data[2] = 0.0f;
    data[3] = 1.0f;
    ed_cluster_id = 0u;
    sc_cluster_id = 0u;
    rgb = 15329511.0f;  // TODO(ben): current equal to (233, 232, 231) for testing
    semantics_rgb = 15329511;
  }

  friend std::ostream& operator << (std::ostream& os, const _SegMatch_PointColorSemantics& p) {
    return os << "x: "<< p.x << ", y: " << p.y << ", z: " << p.z
        << ", EuclideanDistance ID: " << p.ed_cluster_id
        << ", SmoothnessConstraints ID: " << p.sc_cluster_id;
  }

  inline _SegMatch_PointColorSemantics operator- (const _SegMatch_PointColorSemantics& pt_other) {
    _SegMatch_PointColorSemantics result;
    result.x = pt_other.x - data[0];
    result.y = pt_other.y - data[1];
    result.z = pt_other.z - data[2];
    return result;
  }

  // X, Y, Z components of the position of the point.
  // Memory layout (4 x 4 bytes): [ x, y, z, _ ]
  // PCL_ADD_POINT4D;
  // PCL_ADD_RGB;
  // Cluster ID fields.
  // Memory layout (4 x 4 bytes): [ ed_cluster_id, sc_cluster_id, _, _ ]
  union {
    struct {
      uint32_t ed_cluster_id;
      uint32_t sc_cluster_id;
    };
    uint32_t data_c[4];
  };
  // Semantics fields.
  // Memory layout (4 x 4 bytes): [ ed_cluster_id, sc_cluster_id, _, _ ]
  union {
    struct {
      uint8_t semantics_r;
      uint8_t semantics_g;
      uint8_t semantics_b;
    };
    // TODO: float or uint32_t?
    uint32_t semantics_rgb;
  };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
// struct _SegMatch_PointColorSemantics {
//   inline _SegMatch_PointColorSemantics(const _SegMatch_PointColorSemantics &p)
//     : data { p.x, p.y, p.z, 1.0f }, ed_cluster_id(p.ed_cluster_id),
//       sc_cluster_id(p.sc_cluster_id) /*, r(p.r), g(p.g), b(p.b), a(p.a) */ , rgb(p.rgb), semantics_rgb(p.semantics_rgb) {
//         // r = 55;//(static_cast<uint32_t>(p.rgb) >> 16) & 0xff;
//         // g = 144; //(static_cast<uint32_t>(p.rgb) >> 8) & 0xff;
//         // b = 145; //static_cast<uint32_t>(p.rgb) & 0xff;
//         // r = p.r;
//         // g = p.g;
//         // b = p.b;
//         // a = 255u;
//   }

//   inline _SegMatch_PointColorSemantics()
//     : data{ 0.0f, 0.0f, 0.0f, 1.0f }
//     , ed_cluster_id(0u)
//     , sc_cluster_id(0u)
//     // , r(222)
//     // , g(222)
//     // , b(222)
//     // , a(255)
//     , rgb(15329511.0f)  // TODO(ben): current equal to (233, 232, 231) for testing
//     , semantics_rgb(15329511)  // TODO(ben): current equal to (233, 232, 231) for testing
//     // , rgb(static_cast<float>((253 << 16) | (9 << 8) | (11)))  // TODO(ben): change this to something sensible
//   {}

//   // TODO: not sure if this is the right constructor. Should replace the PointXYZRGB(int r, int g, int b) constructor but make sure 1) semantics aren't needed here yet
//   inline _SegMatch_PointColorSemantics(int r, int g, int b)
//     : data{ 0.0f, 0.0f, 0.0f, 1.0f }
//     , ed_cluster_id(0u)
//     , sc_cluster_id(0u)
//     // , r(222)
//     // , g(222)
//     // , b(222)
//     // , a(255)
//     , rgb(15329511.0f)  // TODO(ben): current equal to (233, 232, 231) for testing
//     , semantics_rgb(15329511)  // TODO(ben): current equal to (233, 232, 231) for testing
//     // , rgb(static_cast<float>((253 << 16) | (9 << 8) | (11)))  // TODO(ben): change this to something sensible
//   {}

//   friend std::ostream& operator << (std::ostream& os, const _SegMatch_PointColorSemantics& p) {
//     return os << "x: "<< p.x << ", y: " << p.y << ", z: " << p.z
//         << ", EuclideanDistance ID: " << p.ed_cluster_id
//         << ", SmoothnessConstraints ID: " << p.sc_cluster_id;
//   }

//   // X, Y, Z components of the position of the point.
//   // Memory layout (4 x 4 bytes): [ x, y, z, _ ]
//   PCL_ADD_POINT4D;
//   PCL_ADD_RGB;
//   // Cluster ID fields.
//   // Memory layout (4 x 4 bytes): [ ed_cluster_id, sc_cluster_id, _, _ ]
//   union {
//     struct {
//       uint32_t ed_cluster_id;
//       uint32_t sc_cluster_id;
//     };
//     uint32_t data_c[4];
//   };
//   // Semantics fields.
//   // Memory layout (4 x 4 bytes): [ ed_cluster_id, sc_cluster_id, _, _ ]
//   union {
//     struct {
//       uint8_t semantics_r;
//       uint8_t semantics_g;
//       uint8_t semantics_b;
//     };
//     // TODO: float or uint32_t?
//     uint32_t semantics_rgb;
//   };

//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// };

// Register the point type.
POINT_CLOUD_REGISTER_POINT_STRUCT (_SegMatch_PointColorSemantics,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                  //  (uint8_t, r, r)
                                  //  (uint8_t, g, g)
                                  //  (uint8_t, b, b)
                                  //  (uint8_t, a, a)
                                   (uint32_t, ed_cluster_id, ed_cluster_id)
                                   (uint32_t, sc_cluster_id, sc_cluster_id)
                                   (float, rgb, rgb)
                                   (float, semantics_rgb, semantics_rgb)
)

namespace segmatch {
  typedef _SegMatch_PointColorSemantics PointColorSemantics;
} // namespace segmatch

#endif // SEGMATCH_POINT_COLOR_SEMANTICS_HPP_
