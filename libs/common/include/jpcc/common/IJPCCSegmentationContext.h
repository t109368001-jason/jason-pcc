#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/common/SegmentationOutputType.h>
#include <jpcc/common/SegmentationType.h>

namespace jpcc {

class IJPCCSegmentationContext {
 public:
  [[nodiscard]] virtual SegmentationType       getSegmentationType() const       = 0;
  [[nodiscard]] virtual SegmentationOutputType getSegmentationOutputType() const = 0;

  [[nodiscard]] virtual const GroupOfFrame& getPclFrames() const              = 0;
  [[nodiscard]] virtual const GroupOfFrame& getDynamicPclFrames() const       = 0;
  [[nodiscard]] virtual const GroupOfFrame& getStaticPclFrames() const        = 0;
  [[nodiscard]] virtual const GroupOfFrame& getStaticAddedPclFrames() const   = 0;
  [[nodiscard]] virtual const GroupOfFrame& getStaticRemovedPclFrames() const = 0;

  [[nodiscard]] virtual GroupOfFrame& getPclFrames()              = 0;
  [[nodiscard]] virtual GroupOfFrame& getDynamicPclFrames()       = 0;
  [[nodiscard]] virtual GroupOfFrame& getStaticPclFrames()        = 0;
  [[nodiscard]] virtual GroupOfFrame& getStaticAddedPclFrames()   = 0;
  [[nodiscard]] virtual GroupOfFrame& getStaticRemovedPclFrames() = 0;
};

}  // namespace jpcc