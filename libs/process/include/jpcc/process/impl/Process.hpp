#pragma once

#include <pcl/filters/extract_indices.h>

namespace jpcc::process {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void split(const FramePtr<PointT>& input,
           const IndicesPtr&       indices,
           const FramePtr<PointT>& output,
           const FramePtr<PointT>& outputNegative) {
  pcl::ExtractIndices<PointT> extractIndices;
  extractIndices.setInputCloud(input);
  extractIndices.setIndices(indices);
  if (outputNegative) {
    extractIndices.setNegative(true);
    extractIndices.filter(*outputNegative);
  }
  if (output) {
    extractIndices.setNegative(false);
    extractIndices.filter(*output);
  }
}

}  // namespace jpcc::process
