#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/common/SegmentationOutputType.h>
#include <jpcc/common/SegmentationType.h>

namespace jpcc {

using PointSegmentation = pcl::PointXYZINormal;

class IJPCCSegmentationContext {
 public:
  [[nodiscard]] virtual SegmentationOutputType getSegmentationOutputType() const = 0;

  [[nodiscard]] virtual const GroupOfFrame&                       getFrames() const              = 0;
  [[nodiscard]] virtual const GroupOfPclFrame<PointSegmentation>& getPclFrames() const           = 0;
  [[nodiscard]] virtual const GroupOfFrame&                       getDynamicFrames() const       = 0;
  [[nodiscard]] virtual const GroupOfFrame&                       getStaticFrames() const        = 0;
  [[nodiscard]] virtual const GroupOfFrame&                       getStaticAddedFrames() const   = 0;
  [[nodiscard]] virtual const GroupOfFrame&                       getStaticRemovedFrames() const = 0;

  [[nodiscard]] virtual GroupOfFrame&                       getFrames()              = 0;
  [[nodiscard]] virtual GroupOfPclFrame<PointSegmentation>& getPclFrames()           = 0;
  [[nodiscard]] virtual GroupOfFrame&                       getDynamicFrames()       = 0;
  [[nodiscard]] virtual GroupOfFrame&                       getStaticFrames()        = 0;
  [[nodiscard]] virtual GroupOfFrame&                       getStaticAddedFrames()   = 0;
  [[nodiscard]] virtual GroupOfFrame&                       getStaticRemovedFrames() = 0;
};

}  // namespace jpcc