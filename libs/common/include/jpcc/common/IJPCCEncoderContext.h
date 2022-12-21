#pragma once

#include <jpcc/common/Common.h>

namespace jpcc {

class IJPCCEncoderContext {
 public:
  [[nodiscard]] virtual const GroupOfFrame&                       getDynamicFrames() const                   = 0;
  [[nodiscard]] virtual const GroupOfFrame&                       getStaticFrames() const                    = 0;
  [[nodiscard]] virtual const GroupOfFrame&                       getStaticAddedFrames() const               = 0;
  [[nodiscard]] virtual const GroupOfFrame&                       getStaticRemovedFrames() const             = 0;
  [[nodiscard]] virtual const std::vector<std::shared_ptr<void>>& getCoderDynamicFrames() const              = 0;
  [[nodiscard]] virtual const std::vector<std::shared_ptr<void>>& getCoderStaticFrames() const               = 0;
  [[nodiscard]] virtual const std::vector<std::shared_ptr<void>>& getCoderStaticAddedFrames() const          = 0;
  [[nodiscard]] virtual const std::vector<std::shared_ptr<void>>& getCoderStaticRemovedFrames() const        = 0;
  [[nodiscard]] virtual const std::vector<std::vector<char>>&     getDynamicEncodedBytesVector() const       = 0;
  [[nodiscard]] virtual const std::vector<std::vector<char>>&     getStaticEncodedBytesVector() const        = 0;
  [[nodiscard]] virtual const std::vector<std::vector<char>>&     getStaticAddedEncodedBytesVector() const   = 0;
  [[nodiscard]] virtual const std::vector<std::vector<char>>&     getStaticRemovedEncodedBytesVector() const = 0;

  [[nodiscard]] virtual GroupOfFrame&                       getDynamicFrames()                   = 0;
  [[nodiscard]] virtual GroupOfFrame&                       getStaticFrames()                    = 0;
  [[nodiscard]] virtual GroupOfFrame&                       getStaticAddedFrames()               = 0;
  [[nodiscard]] virtual GroupOfFrame&                       getStaticRemovedFrames()             = 0;
  [[nodiscard]] virtual std::vector<std::shared_ptr<void>>& getCoderDynamicFrames()              = 0;
  [[nodiscard]] virtual std::vector<std::shared_ptr<void>>& getCoderStaticFrames()               = 0;
  [[nodiscard]] virtual std::vector<std::shared_ptr<void>>& getCoderStaticAddedFrames()          = 0;
  [[nodiscard]] virtual std::vector<std::shared_ptr<void>>& getCoderStaticRemovedFrames()        = 0;
  [[nodiscard]] virtual std::vector<std::vector<char>>&     getDynamicEncodedBytesVector()       = 0;
  [[nodiscard]] virtual std::vector<std::vector<char>>&     getStaticEncodedBytesVector()        = 0;
  [[nodiscard]] virtual std::vector<std::vector<char>>&     getStaticAddedEncodedBytesVector()   = 0;
  [[nodiscard]] virtual std::vector<std::vector<char>>&     getStaticRemovedEncodedBytesVector() = 0;
};

}  // namespace jpcc