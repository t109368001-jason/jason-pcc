#pragma once

#include <jpcc/segmentation/JPCCSegmentation.h>
#include <jpcc/segmentation/JPCCSegmentationParameter.h>

namespace jpcc::segmentation {

class JPCCSegmentationAdapter {
 public:
  template <typename PointT>
  [[nodiscard]] static typename JPCCSegmentation<PointT>::Ptr build(const JPCCSegmentationParameter& parameter,
                                                                    int                              startFrameNumber);
};

}  // namespace jpcc::segmentation

#include <jpcc/segmentation/impl/JPCCSegmentationAdapter.hpp>
