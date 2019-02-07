#include "segmatch/search/impl/semantic_kdtree_flann.hpp"


namespace search {
    template class SemanticKdTreeFLANN<segmatch::MapPoint>;
    template class SemanticKdTreeFLANN<segmatch::MapPoint, typename search::L2_Color<float>>;
}
