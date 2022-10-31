#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/common/SegmentationOutputType.h>
#include <jpcc/common/SegmentationType.h>

namespace jpcc {

class IJPCCCombinationContext {
 public:
  [[nodiscard]] virtual SegmentationType       getSegmentationType() const       = 0;
  [[nodiscard]] virtual SegmentationOutputType getSegmentationOutputType() const = 0;

  [[nodiscard]] virtual const GroupOfFrame& getDynamicReconstructPclFrames() const       = 0;
  [[nodiscard]] virtual const GroupOfFrame& getStaticReconstructPclFrames() const        = 0;
  [[nodiscard]] virtual const GroupOfFrame& getStaticAddedReconstructPclFrames() const   = 0;
  [[nodiscard]] virtual const GroupOfFrame& getStaticRemovedReconstructPclFrames() const = 0;
  [[nodiscard]] virtual const GroupOfFrame& getReconstructPclFrames() const              = 0;

  [[nodiscard]] virtual GroupOfFrame& getDynamicReconstructPclFrames()       = 0;
  [[nodiscard]] virtual GroupOfFrame& getStaticReconstructPclFrames()        = 0;
  [[nodiscard]] virtual GroupOfFrame& getStaticAddedReconstructPclFrames()   = 0;
  [[nodiscard]] virtual GroupOfFrame& getStaticRemovedReconstructPclFrames() = 0;
  [[nodiscard]] virtual GroupOfFrame& getReconstructPclFrames()              = 0;
};

}  // namespace jpcc