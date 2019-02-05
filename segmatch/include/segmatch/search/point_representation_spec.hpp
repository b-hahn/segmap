#ifndef SEGMATCH_POINT_REPRESENTATION_SPEC_H_
#define SEGMATCH_POINT_REPRESENTATION_SPEC_H_

#include <pcl/point_representation.h>

#include "segmatch/common.hpp"


namespace pcl {

// template<>
// class DefaultPointRepresentation<segmatch::PointExtended> : public PointRepresentation<segmatch::PointExtended>
// // class DefaultPointRepresentation<segmatch::MapPoint> : public PointRepresentation<segmatch::MapPoint>
// {
//   public:
//     DefaultPointRepresentation()
//     {
//         nr_dimensions_ = 4;
//         trivial_ = true;
//         another_test = 56;
//         std::cout << "---------------> correct template spec!!!!!!!!!!!!\n";
//     }

//     // TODO: potentially add functionality to convert rgb to hue here since that's all we need for segmentation. 
//     virtual void copyToFloatArray(const segmatch::MapPoint& p, float* out) const
//     {
//         out[0] = p.x;
//         out[1] = p.y;
//         out[2] = p.z;
//         out[3] = p.rgb;
//         std::cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ p.rgb = " << out[3];
//     }

//     void printNumDims() const {
//         std::cout << "num dims from inside class: " << nr_dimensions_ << std::endl;
//     }

//     static constexpr int existance = 447;
//     int another_test = 43;
// };


// derive DefaultPointRepresentation class
// TODO: maybe create generic base class and create specialization for color and/or color + semantics
template <typename PointDefault>
  class SemanticPointRepresentation : public PointRepresentation<PointDefault>
  {
    using PointRepresentation<PointDefault>::nr_dimensions_;
    using PointRepresentation<PointDefault>::trivial_;
    // using PointRepresentation<PointDefault>::makeShared;

    public:
      // Boost shared pointers
      typedef boost::shared_ptr<SemanticPointRepresentation<PointDefault> > Ptr;
      typedef boost::shared_ptr<const SemanticPointRepresentation<PointDefault> > ConstPtr;

      SemanticPointRepresentation()
      {
        nr_dimensions_ = 7;
        trivial_ = true;
      }

      virtual ~SemanticPointRepresentation () {}


      inline Ptr
      makeShared () const
      {
        return (Ptr (new SemanticPointRepresentation<PointDefault> (*this)));
      }

      virtual void
      copyToFloatArray (const PointDefault &p, float * out) const
      {
        // If point type is unknown, treat it as a struct/array of floats
        const float* ptr = reinterpret_cast<const float*> (&p);
        for (int i = 0; i < nr_dimensions_; ++i)
          out[i] = ptr[i];
      }
  };


  template<>
  class SemanticPointRepresentation<segmatch::MapPoint> : public PointRepresentation<segmatch::MapPoint>
  {
    public:
      SemanticPointRepresentation()
      {
          nr_dimensions_ = 4;
          trivial_ = true;
      }

      // TODO: potentially add functionality to convert rgb to hue here since that's all we need for segmentation.
      virtual void copyToFloatArray(const segmatch::MapPoint& p, float* out) const
      {
          out[0] = p.x;
          out[1] = p.y;
          out[2] = p.z;
          out[3] = p.rgb;
      }

      void printNumDims() const { std::cout << "num dims from inside class: " << nr_dimensions_ << std::endl; }

      static constexpr int existance = 447;
      int another_test = 43;
  };
  } // namespace pcl

#endif // SEGMATCH_POINT_REPRESENTATION_SPEC_H_
