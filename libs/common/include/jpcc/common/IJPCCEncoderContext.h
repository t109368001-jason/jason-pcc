#pragma once

#include <fstream>

#include <jpcc/common/Common.h>

namespace jpcc {

class IJPCCEncoderContext {
 public:
  [[nodiscard]] virtual const GroupOfFrame&                       getFrames() const      = 0;
  [[nodiscard]] virtual const std::vector<std::shared_ptr<void>>& getCoderFrames() const = 0;
  [[nodiscard]] virtual const std::ostream&                       getOs() const          = 0;

  [[nodiscard]] virtual GroupOfFrame&                       getFrames()      = 0;
  [[nodiscard]] virtual std::vector<std::shared_ptr<void>>& getCoderFrames() = 0;
  [[nodiscard]] virtual std::ostream&                       getOs()          = 0;
};

}  // namespace jpcc