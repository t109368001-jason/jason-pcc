#pragma once

#include <jpcc/common/Common.h>

namespace jpcc {

class IJPCCDecoderContext {
 public:
  [[nodiscard]] virtual const std::vector<shared_ptr<void>>& getDynamicReconstructFrames() const          = 0;
  [[nodiscard]] virtual const std::vector<shared_ptr<void>>& getStaticReconstructFrames() const           = 0;
  [[nodiscard]] virtual const std::vector<shared_ptr<void>>& getStaticAddedReconstructFrames() const      = 0;
  [[nodiscard]] virtual const std::vector<shared_ptr<void>>& getStaticRemovedReconstructFrames() const    = 0;
  [[nodiscard]] virtual const GroupOfFrame&                  getDynamicReconstructPclFrames() const       = 0;
  [[nodiscard]] virtual const GroupOfFrame&                  getStaticReconstructPclFrames() const        = 0;
  [[nodiscard]] virtual const GroupOfFrame&                  getStaticAddedReconstructPclFrames() const   = 0;
  [[nodiscard]] virtual const GroupOfFrame&                  getStaticRemovedReconstructPclFrames() const = 0;

  [[nodiscard]] virtual std::vector<shared_ptr<void>>& getDynamicReconstructFrames()          = 0;
  [[nodiscard]] virtual std::vector<shared_ptr<void>>& getStaticReconstructFrames()           = 0;
  [[nodiscard]] virtual std::vector<shared_ptr<void>>& getStaticAddedReconstructFrames()      = 0;
  [[nodiscard]] virtual std::vector<shared_ptr<void>>& getStaticRemovedReconstructFrames()    = 0;
  [[nodiscard]] virtual GroupOfFrame&                  getDynamicReconstructPclFrames()       = 0;
  [[nodiscard]] virtual GroupOfFrame&                  getStaticReconstructPclFrames()        = 0;
  [[nodiscard]] virtual GroupOfFrame&                  getStaticAddedReconstructPclFrames()   = 0;
  [[nodiscard]] virtual GroupOfFrame&                  getStaticRemovedReconstructPclFrames() = 0;
};

}  // namespace jpcc