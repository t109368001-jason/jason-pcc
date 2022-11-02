#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/common/SegmentationOutputType.h>
#include <jpcc/common/SegmentationType.h>

namespace jpcc {

class IJPCCCombinationContext {
 public:
  [[nodiscard]] virtual SegmentationType       getSegmentationType() const       = 0;
  [[nodiscard]] virtual SegmentationOutputType getSegmentationOutputType() const = 0;

  [[nodiscard]] virtual const GroupOfFrame& getDynamicReconstructFrames() const       = 0;
  [[nodiscard]] virtual const GroupOfFrame& getStaticReconstructFrames() const        = 0;
  [[nodiscard]] virtual const GroupOfFrame& getStaticAddedReconstructFrames() const   = 0;
  [[nodiscard]] virtual const GroupOfFrame& getStaticRemovedReconstructFrames() const = 0;
  [[nodiscard]] virtual const GroupOfFrame& getReconstructFrames() const              = 0;

  [[nodiscard]] virtual GroupOfFrame& getDynamicReconstructFrames()       = 0;
  [[nodiscard]] virtual GroupOfFrame& getStaticReconstructFrames()        = 0;
  [[nodiscard]] virtual GroupOfFrame& getStaticAddedReconstructFrames()   = 0;
  [[nodiscard]] virtual GroupOfFrame& getStaticRemovedReconstructFrames() = 0;
  [[nodiscard]] virtual GroupOfFrame& getReconstructFrames()              = 0;
};

}  // namespace jpcc