#ifndef SEGMATCH_POINT_REPRESENTATION_SPEC_H_
#define SEGMATCH_POINT_REPRESENTATION_SPEC_H_

#include <pcl/point_representation.h>

#include "segmatch/common.hpp"


namespace pcl {

template<>
class DefaultPointRepresentation<segmatch::PointExtended> : public PointRepresentation<segmatch::PointExtended>
// class DefaultPointRepresentation<segmatch::MapPoint> : public PointRepresentation<segmatch::MapPoint>
{
  public:
    DefaultPointRepresentation()
    {
        nr_dimensions_ = 4;
        trivial_ = true;
        another_test = 56;
        std::cout << "---------------> correct template spec!!!!!!!!!!!!\n";
    }

    // TODO: potentially add functionality to convert rgb to hue here since that's all we need for segmentation. 
    virtual void copyToFloatArray(const segmatch::MapPoint& p, float* out) const
    {
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        out[3] = p.rgb;
        std::cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ p.rgb = " << out[3];
    }

    void printNumDims() const {
        std::cout << "num dims from inside class: " << nr_dimensions_ << std::endl;
    }

    static constexpr int existance = 447;
    int another_test = 43;
};
}


#endif // SEGMATCH_POINT_REPRESENTATION_SPEC_H_
