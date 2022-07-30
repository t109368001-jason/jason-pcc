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
  using GroupOfFrameMapPtr = shared_ptr<GroupOfFrameMap<PointT>>;
  using Filter             = pcl::FilterIndices<PointT>;
  using FilterPtr          = typename Filter::Ptr;

 protected:
  PreProcessParameter param_;

 public:
  PreProcessor(PreProcessParameter param);

  void process(GroupOfFrame<PointT>&     groupOfFrame,
               const GroupOfFrameMapPtr& removedMap = nullptr,
               bool                      parallel   = false) const;

  [[nodiscard]] FilterPtr buildFilter(const std::string& algorithm) const;

  void applyAlgorithm(const std::string&    algorithm,
                      GroupOfFrame<PointT>& groupOfFrame,
                      GroupOfFrame<PointT>& removed,
                      bool                  parallel = false) const;

  void applyAlgorithm(const std::string&      algorithm,
                      const FramePtr<PointT>& frame,
                      const FramePtr<PointT>& removed = nullptr) const;
};

}  // namespace jpcc::process

#include <jpcc/process/impl/PreProcessor.hpp>
