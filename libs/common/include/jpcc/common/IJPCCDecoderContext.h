#pragma once

#include <jpcc/common/Common.h>

namespace jpcc {

template <typename PointT>
class IJPCCDecoderContext {
 public:
  [[nodiscard]] virtual const std::vector<shared_ptr<void>>& getDynamicReconstructFrames() const          = 0;
  [[nodiscard]] virtual const std::vector<shared_ptr<void>>& getStaticReconstructFrames() const           = 0;
  [[nodiscard]] virtual const std::vector<shared_ptr<void>>& getStaticAddedReconstructFrames() const      = 0;
  [[nodiscard]] virtual const std::vector<shared_ptr<void>>& getStaticRemovedReconstructFrames() const    = 0;
  [[nodiscard]] virtual const GroupOfFrame<PointT>&          getDynamicReconstructPclFrames() const       = 0;
  [[nodiscard]] virtual const GroupOfFrame<PointT>&          getStaticReconstructPclFrames() const        = 0;
  [[nodiscard]] virtual const GroupOfFrame<PointT>&          getStaticAddedReconstructPclFrames() const   = 0;
  [[nodiscard]] virtual const GroupOfFrame<PointT>&          getStaticRemovedReconstructPclFrames() const = 0;

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