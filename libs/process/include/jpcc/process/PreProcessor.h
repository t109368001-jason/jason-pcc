#pragma once

#include <map>

#include <jpcc/common/GroupOfFrame.h>
#include <jpcc/process/PreProcessParameter.h>

namespace jpcc::process {

template <class PointT = Point>
class PreProcessor {
 public:
  using Ptr                = shared_ptr<PreProcessor>;
  using Frame              = Frame<PointT>;
  using FramePtr           = typename Frame::Ptr;
  using GroupOfFrame       = GroupOfFrame<PointT>;
  using GroupOfFrameMap    = std::map<std::string, GroupOfFrame>;
  using GroupOfFrameMapPtr = shared_ptr<GroupOfFrameMap>;

 protected:
  PreProcessParameter param_;

 public:
  PreProcessor(const PreProcessParameter& param);

  void process(GroupOfFrame& groupOfFrame, GroupOfFrameMapPtr removed = nullptr, bool parallel = false);

  void radiusOutlierRemoval(FramePtr& frame, FramePtr removed = nullptr);

  void radiusOutlierRemoval(GroupOfFrame& groupOfFrame, GroupOfFrameMapPtr removed = nullptr, bool parallel = false);
};

}  // namespace jpcc::process

#include <jpcc/process/impl/PreProcessor.hpp>
