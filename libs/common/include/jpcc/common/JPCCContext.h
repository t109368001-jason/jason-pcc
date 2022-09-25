#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/common/SegmentationType.h>
#include <jpcc/common/SegmentationOutputType.h>
#include <jpcc/common/CoderBackendType.h>

namespace jpcc {

template <typename PointT>
struct JPCCContext {
  SegmentationType       segmentationType;
  SegmentationOutputType segmentationOutputType;

  GroupOfFrame<PointT> pclFrames;

  GroupOfFrame<PointT> dynamicPclFrames;
  GroupOfFrame<PointT> staticPclFrames;
  GroupOfFrame<PointT> staticAddedPclFrames;
  GroupOfFrame<PointT> staticRemovedPclFrames;

  // coder specified type
  std::vector<shared_ptr<void>> dynamicFrames;
  std::vector<shared_ptr<void>> staticFrames;
  std::vector<shared_ptr<void>> staticAddedFrames;
  std::vector<shared_ptr<void>> staticRemovedFrames;

  std::vector<char> dynamicEncodedBytes;
  std::vector<char> staticEncodedBytes;
  std::vector<char> staticAddedEncodedBytes;
  std::vector<char> staticRemovedEncodedBytes;

  // coder specified type
  std::vector<shared_ptr<void>> dynamicReconstructFrames;
  std::vector<shared_ptr<void>> staticReconstructFrames;
  std::vector<shared_ptr<void>> staticAddedReconstructFrames;
  std::vector<shared_ptr<void>> staticRemovedReconstructFrames;

  GroupOfFrame<PointT> dynamicReconstructPclFrames;
  GroupOfFrame<PointT> staticReconstructPclFrames;
  GroupOfFrame<PointT> staticAddedReconstructPclFrames;
  GroupOfFrame<PointT> staticRemovedReconstructPclFrames;

  GroupOfFrame<PointT> reconstructPclFrames;

  void init(size_t frameCount);

  void resize(size_t frameCount);

  void clear();
};

}  // namespace jpcc

#include <jpcc/common/impl/JPCCContext.hpp>
