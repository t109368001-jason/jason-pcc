#pragma once

#include <map>

#include <jpcc/common/GroupOfFrame.h>
#include <jpcc/process/PreProcessParameter.h>

namespace jpcc::process {

template <class PointT = Point>
class PreProcessor {
 public:
  using Ptr                = shared_ptr<PreProcessor>;
  using Frame              = jpcc::Frame<PointT>;
  using FramePtr           = typename Frame::Ptr;
  using GroupOfFrame       = jpcc::GroupOfFrame<PointT>;
  using GroupOfFramePtr    = shared_ptr<GroupOfFrame>;
  using GroupOfFrameMap    = std::map<std::string, GroupOfFramePtr>;
  using GroupOfFrameMapPtr = shared_ptr<GroupOfFrameMap>;

 protected:
  PreProcessParameter param_;

 public:
  PreProcessor(PreProcessParameter param);

  void process(GroupOfFrame& groupOfFrame, GroupOfFrameMapPtr removed = nullptr, bool parallel = false);

  void radiusOutlierRemoval(FramePtr& frame, FramePtr removed = nullptr);

  void radiusOutlierRemoval(GroupOfFrame& groupOfFrame, GroupOfFramePtr removed = nullptr, bool parallel = false);

  void statisticalOutlierRemoval(FramePtr& frame, FramePtr removed = nullptr);

  void statisticalOutlierRemoval(GroupOfFrame& groupOfFrame, GroupOfFramePtr removed = nullptr, bool parallel = false);
};

}  // namespace jpcc::process

#include <jpcc/process/impl/PreProcessor.hpp>
