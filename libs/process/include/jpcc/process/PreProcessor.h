#pragma once

#include <map>
#include <string>

#include <pcl/filters/filter_indices.h>

#include <jpcc/common/Common.h>
#include <jpcc/process/PreProcessParameter.h>

namespace jpcc::process {

template <typename PointT>
class PreProcessor {
 public:
  using Ptr                = shared_ptr<PreProcessor>;
  using Frame              = jpcc::Frame<PointT>;
  using FramePtr           = typename Frame::Ptr;
  using GroupOfFrame       = jpcc::GroupOfFrame<PointT>;
  using GroupOfFrameMap    = std::map<std::string, GroupOfFrame>;
  using GroupOfFrameMapPtr = shared_ptr<GroupOfFrameMap>;
  using Filter             = pcl::FilterIndices<PointT>;
  using FilterPtr          = typename Filter::Ptr;

 protected:
  PreProcessParameter param_;

 public:
  PreProcessor(PreProcessParameter param);

  void process(GroupOfFrame& groupOfFrame, GroupOfFrameMapPtr removedMap = nullptr, bool parallel = false) const;

  [[nodiscard]] FilterPtr buildFilter(const std::string& algorithm) const;

  void applyAlgorithm(const std::string& algorithm,
                      GroupOfFrame&      groupOfFrame,
                      GroupOfFrame&      removed  = nullptr,
                      bool               parallel = false) const;

  void applyAlgorithm(const std::string& algorithm, FramePtr frame, FramePtr removed = nullptr) const;
};

}  // namespace jpcc::process

#include <jpcc/process/impl/PreProcessor.hpp>
