#pragma once

#include <jpcc/common/Common.h>

namespace jpcc {

template <typename PointT>
class IJPCCEncoderContext {
 public:
  [[nodiscard]] virtual const GroupOfFrame<PointT>&           getDynamicPclFrames() const                = 0;
  [[nodiscard]] virtual const GroupOfFrame<PointT>&           getStaticPclFrames() const                 = 0;
  [[nodiscard]] virtual const GroupOfFrame<PointT>&           getStaticAddedPclFrames() const            = 0;
  [[nodiscard]] virtual const GroupOfFrame<PointT>&           getStaticRemovedPclFrames() const          = 0;
  [[nodiscard]] virtual const std::vector<shared_ptr<void>>&  getDynamicFrames() const                   = 0;
  [[nodiscard]] virtual const std::vector<shared_ptr<void>>&  getStaticFrames() const                    = 0;
  [[nodiscard]] virtual const std::vector<shared_ptr<void>>&  getStaticAddedFrames() const               = 0;
  [[nodiscard]] virtual const std::vector<shared_ptr<void>>&  getStaticRemovedFrames() const             = 0;
  [[nodiscard]] virtual const std::vector<std::vector<char>>& getDynamicEncodedBytesVector() const       = 0;
  [[nodiscard]] virtual const std::vector<std::vector<char>>& getStaticEncodedBytesVector() const        = 0;
  [[nodiscard]] virtual const std::vector<std::vector<char>>& getStaticAddedEncodedBytesVector() const   = 0;
  [[nodiscard]] virtual const std::vector<std::vector<char>>& getStaticRemovedEncodedBytesVector() const = 0;

  [[nodiscard]] virtual GroupOfFrame<PointT>&           getDynamicPclFrames()                = 0;
  [[nodiscard]] virtual GroupOfFrame<PointT>&           getStaticPclFrames()                 = 0;
  [[nodiscard]] virtual GroupOfFrame<PointT>&           getStaticAddedPclFrames()            = 0;
  [[nodiscard]] virtual GroupOfFrame<PointT>&           getStaticRemovedPclFrames()          = 0;
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