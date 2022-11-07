#pragma once

#include <map>
#include <string>

#include <pcl/filters/filter_indices.h>

#include <jpcc/common/Common.h>
#include <jpcc/process/PreProcessParameter.h>

namespace jpcc::process {

class PreProcessor {
 public:
  using Ptr                = shared_ptr<PreProcessor>;
  using GroupOfFrameMapPtr = shared_ptr<GroupOfFrameMap>;

 protected:
  PreProcessParameter param_;

 public:
  PreProcessor(PreProcessParameter param);  // NOLINT(google-explicit-constructor)

  void process(GroupOfFrame& groupOfFrame, const GroupOfFrameMapPtr& removedMap = nullptr, bool parallel = false) const;

  void applyAlgorithm(const std::string& algorithm,
                      GroupOfFrame&      groupOfFrame,
                      GroupOfFrame&      removed,
                      bool               parallel = false) const;

  void applyAlgorithm(const std::string& algorithm, const FramePtr& frame, const FramePtr& removed = nullptr) const;
};

void conditionalRemoval(Frame& frame, const Condition& condition, Indices& indices);

void conditionalRemoval(Frame& frame, const Condition& condition, Indices& indices, Indices& removedIndices);

}  // namespace jpcc::process
