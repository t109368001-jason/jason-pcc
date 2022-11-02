#pragma once

#include <jpcc/segmentation/JPCCSegmentation.h>
#include <jpcc/segmentation/JPCCSegmentationParameter.h>

namespace jpcc::segmentation {

class JPCCSegmentationAdapter {
 public:
  [[nodiscard]] static typename JPCCSegmentation::Ptr build(const JPCCSegmentationParameter& parameter,
                                                            int                              startFrameNumber);
};

}  // namespace jpcc::segmentation
