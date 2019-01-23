#include "segmatch/points_neighbors_providers/impl/semantic_kdtree_points_neighbors_provider.hpp"

namespace segmatch {
// Instantiate SemanticKdTreePointsNeighborsProvider for the template parameters used in the
// application.
template class SemanticKdTreePointsNeighborsProvider<MapPoint>;
// Add any other required instantiation here or in a separate file and declare them in
// segmatch/points_neighbors_providers/impl/semantic_kdtree_points_neighbors_provider.hpp.
} // namespace segmatch
