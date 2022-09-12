#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/common/SegmentationOutputType.h>
#include <jpcc/common/CoderBackendType.h>

namespace jpcc {

template <typename PointT>
struct JPCCContext {
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

  std::vector<std::vector<char>> dynamicEncodedFramesBytes;
  std::vector<std::vector<char>> staticEncodedFramesBytes;
  std::vector<std::vector<char>> staticAddedEncodedFramesBytes;
  std::vector<std::vector<char>> staticRemovedEncodedFramesBytes;

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

  void clear();
};

}  // namespace jpcc

#include <jpcc/common/impl/JPCCContext.hpp>
