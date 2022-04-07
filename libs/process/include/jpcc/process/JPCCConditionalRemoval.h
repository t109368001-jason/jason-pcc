#pragma once

#define PCL_NO_PRECOMPILE
#include <pcl/filters/filter_indices.h>

#include <jpcc/common/Common.h>
#include <jpcc/process/JPCCConditionalRemovalParameter.h>

namespace jpcc::process {

template <typename PointT>
class JPCCConditionalRemoval : public virtual pcl::FilterIndices<PointT> {
 public:
  using Ptr      = shared_ptr<JPCCConditionalRemoval>;
  using Frame    = jpcc::Frame<PointT>;
  using FramePtr = typename Frame::Ptr;

 protected:
  JPCCConditionalRemovalParameter param_;

 public:
  JPCCConditionalRemoval(JPCCConditionalRemovalParameter param, int extract_removed_indices = false);

 protected:
  void applyFilter(pcl::Indices& indices) override;
};

}  // namespace jpcc::process

#include <jpcc/process/impl/JPCCConditionalRemoval.hpp>
