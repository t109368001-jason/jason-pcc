#pragma once

#include <jpcc/common/Common.h>

namespace jpcc {

class IJPCCDecoderContext {
 public:
  [[nodiscard]] virtual Index               getStartFrameNumber() const               = 0;
  [[nodiscard]] virtual const GroupOfFrame& getDynamicReconstructFrames() const       = 0;
  [[nodiscard]] virtual const GroupOfFrame& getStaticReconstructFrames() const        = 0;
  [[nodiscard]] virtual const GroupOfFrame& getStaticAddedReconstructFrames() const   = 0;
  [[nodiscard]] virtual const GroupOfFrame& getStaticRemovedReconstructFrames() const = 0;

  [[nodiscard]] virtual Index&        getStartFrameNumber()               = 0;
  [[nodiscard]] virtual GroupOfFrame& getDynamicReconstructFrames()       = 0;
  [[nodiscard]] virtual GroupOfFrame& getStaticReconstructFrames()        = 0;
  [[nodiscard]] virtual GroupOfFrame& getStaticAddedReconstructFrames()   = 0;
  [[nodiscard]] virtual GroupOfFrame& getStaticRemovedReconstructFrames() = 0;
};

}  // namespace jpcc