#include "segmatch/descriptors/descriptors.hpp"

#include <Eigen/Dense>
#include <glog/logging.h>
#include <laser_slam/common.hpp>

#include "segmatch/descriptors/cnn.hpp"
#include "segmatch/descriptors/eigenvalue_based.hpp"
#include "segmatch/descriptors/ensemble_shape_functions.hpp"

namespace segmatch {

Descriptor::Descriptor() {}

Descriptor::~Descriptor() {}

void Descriptor::describe(Segment* segment_ptr) {
  describe(*segment_ptr, &segment_ptr->getLastView().features);
}

// Descriptors methods definition
Descriptors::Descriptors() {
  //TODO: Implement default construction from default parameters.
  CHECK(false) << "Construction of descriptors without parameters is not implemented.";
}

Descriptors::~Descriptors() {}

Descriptors::Descriptors(const DescriptorsParameters& parameters/* , bool use_color, bool use_semantic_segmentation */) {
  CHECK_GT(parameters.descriptor_types.size(), 0) << "Description impossible without a descriptor.";
//   LOG(INFO) << "--------------------> in Descriptors ctor: " << use_color << " and " << use_semantic_segmentation;
  // Create the descriptors.
  for (size_t i = 0u; i < parameters.descriptor_types.size(); ++i) {
    if (parameters.descriptor_types[i] == "EigenvalueBased") {
      descriptors_.push_back(std::unique_ptr<Descriptor>(
          new EigenvalueBasedDescriptor(parameters)));
    } else if (parameters.descriptor_types[i] == "EnsembleShapeFunctions") {
      descriptors_.push_back(std::unique_ptr<Descriptor>(new EnsembleShapeFunctions(parameters)));
    } else if (parameters.descriptor_types[i] == "CNN") {
      descriptors_.push_back(std::unique_ptr<Descriptor>(new CNNDescriptor(parameters/* , use_color, use_semantic_segmentation */)));
      use_color_ = parameters.use_color;
      use_semantic_segmentation_ = parameters.use_semantic_segmentation;
    } else {
      CHECK(false) << "The descriptor '" << parameters.descriptor_types[i] <<
          "' was not implemented.";
    }
  }
}

void Descriptors::describe(Segment* segment_ptr) {
  describe(*segment_ptr, &segment_ptr->getLastView().features);
}

void Descriptors::describe(const Segment& segment, Features* features) {
  CHECK_NOTNULL(features)->clear();
  CHECK_GT(descriptors_.size(), 0) << "Description impossible without a descriptor.";
  for (size_t i = 0u; i < descriptors_.size(); ++i) {
    descriptors_[i]->describe(segment, features);
  }
}

void Descriptors::describe(SegmentedCloud* segmented_cloud_ptr) {
  CHECK_NOTNULL(segmented_cloud_ptr);
  CHECK_GT(descriptors_.size(), 0) << "Description impossible without a descriptor.";
  LOG(INFO) << "--------------------> " << use_color_ << " and " << use_semantic_segmentation_;
  for (size_t i = 0u; i < descriptors_.size(); ++i) {
    descriptors_[i]->describe(segmented_cloud_ptr, use_color_, use_semantic_segmentation_);
  }
}

unsigned int Descriptors::dimension() const {
  CHECK_GT(descriptors_.size(), 0) << "Description impossible without a descriptor.";
  unsigned int dimension = 0;
  for (size_t i = 0u; i < descriptors_.size(); ++i) {
    dimension += descriptors_[i]->dimension();
  }
  return dimension;
}

} // namespace segmatch
