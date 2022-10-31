#pragma once

#include <jpcc/common/Common.h>

namespace jpcc {

class IJPCCEncoderContext {
 public:
  [[nodiscard]] virtual const GroupOfFrame&                   getDynamicPclFrames() const                = 0;
  [[nodiscard]] virtual const GroupOfFrame&                   getStaticPclFrames() const                 = 0;
  [[nodiscard]] virtual const GroupOfFrame&                   getStaticAddedPclFrames() const            = 0;
  [[nodiscard]] virtual const GroupOfFrame&                   getStaticRemovedPclFrames() const          = 0;
  [[nodiscard]] virtual const std::vector<shared_ptr<void>>&  getDynamicFrames() const                   = 0;
  [[nodiscard]] virtual const std::vector<shared_ptr<void>>&  getStaticFrames() const                    = 0;
  [[nodiscard]] virtual const std::vector<shared_ptr<void>>&  getStaticAddedFrames() const               = 0;
  [[nodiscard]] virtual const std::vector<shared_ptr<void>>&  getStaticRemovedFrames() const             = 0;
  [[nodiscard]] virtual const std::vector<std::vector<char>>& getDynamicEncodedBytesVector() const       = 0;
  [[nodiscard]] virtual const std::vector<std::vector<char>>& getStaticEncodedBytesVector() const        = 0;
  [[nodiscard]] virtual const std::vector<std::vector<char>>& getStaticAddedEncodedBytesVector() const   = 0;
  [[nodiscard]] virtual const std::vector<std::vector<char>>& getStaticRemovedEncodedBytesVector() const = 0;

  [[nodiscard]] virtual GroupOfFrame&                   getDynamicPclFrames()                = 0;
  [[nodiscard]] virtual GroupOfFrame&                   getStaticPclFrames()                 = 0;
  [[nodiscard]] virtual GroupOfFrame&                   getStaticAddedPclFrames()            = 0;
  [[nodiscard]] virtual GroupOfFrame&                   getStaticRemovedPclFrames()          = 0;
  [[nodiscard]] virtual std::vector<shared_ptr<void>>&  getDynamicFrames()                   = 0;
  [[nodiscard]] virtual std::vector<shared_ptr<void>>&  getStaticFrames()                    = 0;
  [[nodiscard]] virtual std::vector<shared_ptr<void>>&  getStaticAddedFrames()               = 0;
  [[nodiscard]] virtual std::vector<shared_ptr<void>>&  getStaticRemovedFrames()             = 0;
  [[nodiscard]] virtual std::vector<std::vector<char>>& getDynamicEncodedBytesVector()       = 0;
  [[nodiscard]] virtual std::vector<std::vector<char>>& getStaticEncodedBytesVector()        = 0;
  [[nodiscard]] virtual std::vector<std::vector<char>>& getStaticAddedEncodedBytesVector()   = 0;
  [[nodiscard]] virtual std::vector<std::vector<char>>& getStaticRemovedEncodedBytesVector() = 0;
};

}  // namespace jpcc