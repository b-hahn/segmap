#include "segmatch/search/impl/semantic_kdtree_flann.hpp"

// #ifndef PCL_NO_PRECOMPILE
// #include <pcl/impl/instantiate.hpp>
// #include <pcl/point_types.h>
// // Instantiations of specific point types
// // PCL_INSTANTIATE(SemanticKdTreeFLANN, PCL_POINT_TYPES)
// #endif    // PCL_NO_PRECOMPILE

namespace search {
    // TODO(ben): which specializatoins do I need?
    template class SemanticKdTreeFLANN<segmatch::MapPoint>;
}