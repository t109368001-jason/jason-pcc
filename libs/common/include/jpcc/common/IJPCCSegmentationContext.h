#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/common/SegmentationOutputType.h>
#include <jpcc/common/SegmentationType.h>

namespace jpcc {

template <typename PointT>
class IJPCCSegmentationContext {
 public:
  [[nodiscard]] virtual const SegmentationType       getSegmentationType() const       = 0;
  [[nodiscard]] virtual const SegmentationOutputType getSegmentationOutputType() const = 0;
  [[nodiscard]] virtual const GroupOfFrame<PointT>&  getPclFrames() const              = 0;
  [[nodiscard]] virtual const GroupOfFrame<PointT>&  getDynamicPclFrames() const       = 0;
  [[nodiscard]] virtual const GroupOfFrame<PointT>&  getStaticPclFrames() const        = 0;
  [[nodiscard]] virtual const GroupOfFrame<PointT>&  getStaticAddedPclFrames() const   = 0;
  [[nodiscard]] virtual const GroupOfFrame<PointT>&  getStaticRemovedPclFrames() const = 0;

  [[nodiscard]] virtual GroupOfFrame<PointT>& getPclFrames()              = 0;
  [[nodiscard]] virtual GroupOfFrame<PointT>& getDynamicPclFrames()       = 0;
  [[nodiscard]] virtual GroupOfFrame<PointT>& getStaticPclFrames()        = 0;
  [[nodiscard]] virtual GroupOfFrame<PointT>& getStaticAddedPclFrames()   = 0;
  [[nodiscard]] virtual GroupOfFrame<PointT>& getStaticRemovedPclFrames() = 0;
};

}  // namespace jpcc