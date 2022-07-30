#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/process/JPCCConditionalRemovalParameter.h>

#define PCL_NO_PRECOMPILE
#include <pcl/filters/filter_indices.h>

namespace jpcc::process {

template <typename PointT>
class JPCCConditionalRemoval : public virtual pcl::FilterIndices<PointT> {
 public:
  using Ptr = shared_ptr<JPCCConditionalRemoval>;

 protected:
  JPCCConditionalRemovalParameter param_;

 public:
  JPCCConditionalRemoval(JPCCConditionalRemovalParameter param, int extract_removed_indices = false);

 protected:
  void applyFilter(Indices& indices) override;
};

}  // namespace jpcc::process

#include <jpcc/process/impl/JPCCConditionalRemoval.hpp>
