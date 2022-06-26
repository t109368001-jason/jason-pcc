#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/octree/OctreeNBuf.h>

#include <jpcc/octree/JPCCOctreePointCloud.h>

namespace jpcc::process {

template <bool singlePointVoxel = false>
class OctreePointCloudOperation
    : public octree::JPCCOctreePointCloud<
          PointXYZINormal,
          typename std::conditional<singlePointVoxel,
                                    pcl::octree::OctreeContainerPointIndex,
                                    pcl::octree::OctreeContainerPointIndices>::type,
          pcl::octree::OctreeContainerEmpty,
          octree::OctreeNBuf<2,
                             typename std::conditional<singlePointVoxel,
                                                       pcl::octree::OctreeContainerPointIndex,
                                                       pcl::octree::OctreeContainerPointIndices>::type,
                             pcl::octree::OctreeContainerEmpty>> {
 public:
  using Base = octree::JPCCOctreePointCloud<
      PointXYZINormal,
      typename std::conditional<singlePointVoxel,
                                pcl::octree::OctreeContainerPointIndex,
                                pcl::octree::OctreeContainerPointIndices>::type,
      pcl::octree::OctreeContainerEmpty,
      octree::OctreeNBuf<2,
                         typename std::conditional<singlePointVoxel,
                                                   pcl::octree::OctreeContainerPointIndex,
                                                   pcl::octree::OctreeContainerPointIndices>::type,
                         pcl::octree::OctreeContainerEmpty>>;
  using OctreeBase = octree::OctreeNBuf<2,
                                        typename std::conditional<singlePointVoxel,
                                                                  pcl::octree::OctreeContainerPointIndex,
                                                                  pcl::octree::OctreeContainerPointIndices>::type,
                                        pcl::octree::OctreeContainerEmpty>;

  using Ptr = shared_ptr<OctreePointCloudOperation>;

 protected:
  FramePtr source_;
  FramePtr target_;

 public:
  OctreePointCloudOperation(double resolution);

  void setSource(FramePtr source);

  void setTarget(FramePtr target);

  [[nodiscard]] FramePtr targetAndNotSource();
};

}  // namespace jpcc::process

#include <jpcc/process/impl/OctreePointCloudOperation.hpp>