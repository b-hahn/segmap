#include "segmatch/segmenters/impl/semantic_segmenter.hpp"
#include "segmatch/common.hpp"

namespace segmatch {
// Instantiate SemanticSegmenter for the template parameters used in the application.
template class SemanticSegmenter<MapPoint>;
// Add any other required instantiation here or in a separate file and declare them in
// segmatch/segmenters/impl/euclidean_segmenter.hpp.
} // namespace segmatch
