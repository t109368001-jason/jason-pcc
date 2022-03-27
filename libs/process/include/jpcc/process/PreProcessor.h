#pragma once

#include <map>

#define PCL_NO_PRECOMPILE
#include <pcl/filters/filter_indices.h>

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
  using Filter             = pcl::FilterIndices<PointT>;
  using FilterPtr          = typename pcl::FilterIndices<PointT>::Ptr;

 protected:
  PreProcessParameter param_;

 public:
  PreProcessor(PreProcessParameter param);

  void process(GroupOfFrame& groupOfFrame, GroupOfFrameMapPtr removed = nullptr, bool parallel = false);

  FilterPtr buildFilter(const std::string& algorithm);

  void applyAlgorithm(const std::string& algorithm,
                      GroupOfFrame&      groupOfFrame,
                      GroupOfFramePtr    removed  = nullptr,
                      bool               parallel = false);

  void applyAlgorithm(const std::string& algorithm, FramePtr frame, FramePtr removed = nullptr);
};

}  // namespace jpcc::process

#include <jpcc/process/impl/PreProcessor.hpp>
