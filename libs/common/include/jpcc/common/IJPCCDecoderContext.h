#pragma once

#include <jpcc/common/Common.h>

namespace jpcc {

class IJPCCDecoderContext {
 public:
  [[nodiscard]] virtual Index                                     getStartFrameNumber() const                    = 0;
  [[nodiscard]] virtual const std::vector<std::shared_ptr<void>>& getCoderDynamicReconstructFrames() const       = 0;
  [[nodiscard]] virtual const std::vector<std::shared_ptr<void>>& getCoderStaticReconstructFrames() const        = 0;
  [[nodiscard]] virtual const std::vector<std::shared_ptr<void>>& getCoderStaticAddedReconstructFrames() const   = 0;
  [[nodiscard]] virtual const std::vector<std::shared_ptr<void>>& getCoderStaticRemovedReconstructFrames() const = 0;
  [[nodiscard]] virtual const GroupOfFrame&                       getDynamicReconstructFrames() const            = 0;
  [[nodiscard]] virtual const GroupOfFrame&                       getStaticReconstructFrames() const             = 0;
  [[nodiscard]] virtual const GroupOfFrame&                       getStaticAddedReconstructFrames() const        = 0;
  [[nodiscard]] virtual const GroupOfFrame&                       getStaticRemovedReconstructFrames() const      = 0;

  [[nodiscard]] virtual Index&                              getStartFrameNumber()                    = 0;
  [[nodiscard]] virtual std::vector<std::shared_ptr<void>>& getCoderDynamicReconstructFrames()       = 0;
  [[nodiscard]] virtual std::vector<std::shared_ptr<void>>& getCoderStaticReconstructFrames()        = 0;
  [[nodiscard]] virtual std::vector<std::shared_ptr<void>>& getCoderStaticAddedReconstructFrames()   = 0;
  [[nodiscard]] virtual std::vector<std::shared_ptr<void>>& getCoderStaticRemovedReconstructFrames() = 0;
  [[nodiscard]] virtual GroupOfFrame&                       getDynamicReconstructFrames()            = 0;
  [[nodiscard]] virtual GroupOfFrame&                       getStaticReconstructFrames()             = 0;
  [[nodiscard]] virtual GroupOfFrame&                       getStaticAddedReconstructFrames()        = 0;
  [[nodiscard]] virtual GroupOfFrame&                       getStaticRemovedReconstructFrames()      = 0;
};

}  // namespace jpcc