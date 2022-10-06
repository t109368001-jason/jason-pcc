#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/common/SegmentationOutputType.h>
#include <jpcc/common/SegmentationType.h>

namespace jpcc {

template <typename PointT>
class IJPCCCombinationContext {
 public:
  [[nodiscard]] virtual const SegmentationType       getSegmentationType() const                  = 0;
  [[nodiscard]] virtual const SegmentationOutputType getSegmentationOutputType() const            = 0;
  [[nodiscard]] virtual const GroupOfFrame<PointT>&  getDynamicReconstructPclFrames() const       = 0;
  [[nodiscard]] virtual const GroupOfFrame<PointT>&  getStaticReconstructPclFrames() const        = 0;
  [[nodiscard]] virtual const GroupOfFrame<PointT>&  getStaticAddedReconstructPclFrames() const   = 0;
  [[nodiscard]] virtual const GroupOfFrame<PointT>&  getStaticRemovedReconstructPclFrames() const = 0;
  [[nodiscard]] virtual const GroupOfFrame<PointT>&  getReconstructPclFrames() const              = 0;

  [[nodiscard]] virtual GroupOfFrame<PointT>& getDynamicReconstructPclFrames()       = 0;
  [[nodiscard]] virtual GroupOfFrame<PointT>& getStaticReconstructPclFrames()        = 0;
  [[nodiscard]] virtual GroupOfFrame<PointT>& getStaticAddedReconstructPclFrames()   = 0;
  [[nodiscard]] virtual GroupOfFrame<PointT>& getStaticRemovedReconstructPclFrames() = 0;
  [[nodiscard]] virtual GroupOfFrame<PointT>& getReconstructPclFrames()              = 0;
};

}  // namespace jpcc