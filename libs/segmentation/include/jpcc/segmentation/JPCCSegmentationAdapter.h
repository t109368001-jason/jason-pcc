#pragma once

#include <jpcc/octree/JPCCOctreePointCloud.h>
#include <jpcc/segmentation/JPCCSegmentation.h>
#include <jpcc/segmentation/JPCCSegmentationParameter.h>
#include <jpcc/segmentation/OctreeContainerSegmentationGMMCenter.h>

namespace jpcc::segmentation {

class JPCCSegmentationAdapter {
 public:
  template <typename PointT>
  static typename JPCCSegmentation<PointT>::Ptr build(const JPCCSegmentationParameter& parameter, int startFrameNumber);
};

}  // namespace jpcc::segmentation

#include <jpcc/segmentation/impl/JPCCSegmentationAdapter.hpp>
