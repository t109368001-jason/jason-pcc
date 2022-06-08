#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/octree/OctreeNBuf.h>

#include <jpcc/octree/JPCCOctreePointCloud.h>

namespace jpcc::process {

template <typename PointT, bool singlePointVoxel = false>
class OctreePointCloudOperation
    : public octree::JPCCOctreePointCloud<
          PointT,
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
      PointT,
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

  using Ptr          = shared_ptr<OctreePointCloudOperation>;
  using Frame        = jpcc::Frame<PointT>;
  using FramePtr     = typename Frame::Ptr;
  using GroupOfFrame = jpcc::GroupOfFrame<PointT>;

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