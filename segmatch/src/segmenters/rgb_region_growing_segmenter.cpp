#include "segmatch/segmenters/impl/rgb_region_growing_segmenter.hpp"
#include "segmatch/common.hpp"

namespace segmatch {
// Instantiate RGBRegionGrowingSegmenter for the template parameters used in the application.
template class RGBRegionGrowingSegmenter<MapPoint>;
// Add any other required instantiation here or in a separate file and declare them in
// segmatch/segmenters/impl/euclidean_segmenter.hpp.
} // namespace segmatch
