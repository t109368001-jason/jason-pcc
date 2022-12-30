#pragma once

#include <fstream>

#include <jpcc/common/Common.h>

namespace jpcc {

class IJPCCDecoderContext {
 public:
  [[nodiscard]] virtual Index                                     getStartFrameNumber() const = 0;
  [[nodiscard]] virtual const std::istream&                       getIs() const               = 0;
  [[nodiscard]] virtual const std::vector<std::shared_ptr<void>>& getCoderFrames() const      = 0;
  [[nodiscard]] virtual const GroupOfFrame&                       getFrames() const           = 0;

  [[nodiscard]] virtual Index&                              getStartFrameNumber() = 0;
  [[nodiscard]] virtual std::istream&                       getIs()               = 0;
  [[nodiscard]] virtual std::vector<std::shared_ptr<void>>& getCoderFrames()      = 0;
  [[nodiscard]] virtual GroupOfFrame&                       getFrames()           = 0;
};

}  // namespace jpcc