#pragma once

#include <jpcc/common/Common.h>

namespace jpcc {

template <typename PointT>
class IJPCCDecoderContext {
 public:
  [[nodiscard]] virtual const std::vector<char>&             getDynamicEncodedBytes() const               = 0;
  [[nodiscard]] virtual const std::vector<char>&             getStaticEncodedBytes() const                = 0;
  [[nodiscard]] virtual const std::vector<char>&             getStaticAddedEncodedBytes() const           = 0;
  [[nodiscard]] virtual const std::vector<char>&             getStaticRemovedEncodedBytes() const         = 0;
  [[nodiscard]] virtual const std::vector<shared_ptr<void>>& getDynamicReconstructFrames() const          = 0;
  [[nodiscard]] virtual const std::vector<shared_ptr<void>>& getStaticReconstructFrames() const           = 0;
  [[nodiscard]] virtual const std::vector<shared_ptr<void>>& getStaticAddedReconstructFrames() const      = 0;
  [[nodiscard]] virtual const std::vector<shared_ptr<void>>& getStaticRemovedReconstructFrames() const    = 0;
  [[nodiscard]] virtual const GroupOfFrame<PointT>&          getDynamicReconstructPclFrames() const       = 0;
  [[nodiscard]] virtual const GroupOfFrame<PointT>&          getStaticReconstructPclFrames() const        = 0;
  [[nodiscard]] virtual const GroupOfFrame<PointT>&          getStaticAddedReconstructPclFrames() const   = 0;
  [[nodiscard]] virtual const GroupOfFrame<PointT>&          getStaticRemovedReconstructPclFrames() const = 0;

  [[nodiscard]] virtual std::vector<char>&             getDynamicEncodedBytes()               = 0;
  [[nodiscard]] virtual std::vector<char>&             getStaticEncodedBytes()                = 0;
  [[nodiscard]] virtual std::vector<char>&             getStaticAddedEncodedBytes()           = 0;
  [[nodiscard]] virtual std::vector<char>&             getStaticRemovedEncodedBytes()         = 0;
  [[nodiscard]] virtual std::vector<shared_ptr<void>>& getDynamicReconstructFrames()          = 0;
  [[nodiscard]] virtual std::vector<shared_ptr<void>>& getStaticReconstructFrames()           = 0;
  [[nodiscard]] virtual std::vector<shared_ptr<void>>& getStaticAddedReconstructFrames()      = 0;
  [[nodiscard]] virtual std::vector<shared_ptr<void>>& getStaticRemovedReconstructFrames()    = 0;
  [[nodiscard]] virtual GroupOfFrame<PointT>&          getDynamicReconstructPclFrames()       = 0;
  [[nodiscard]] virtual GroupOfFrame<PointT>&          getStaticReconstructPclFrames()        = 0;
  [[nodiscard]] virtual GroupOfFrame<PointT>&          getStaticAddedReconstructPclFrames()   = 0;
  [[nodiscard]] virtual GroupOfFrame<PointT>&          getStaticRemovedReconstructPclFrames() = 0;
};

}  // namespace jpcc