#pragma once

#include <jpcc/common/JPCCHeader.h>
#include <jpcc/common/Common.h>
#include <jpcc/common/IJPCCCombinationContext.h>
#include <jpcc/octree/JPCCOctreePointCloud.h>
#include <jpcc/octree/OctreeContainerEditableIndex.h>

namespace jpcc::combination {

class JPCCCombination {
 protected:
  PclFramePtr<pcl::PointXYZ>                                                                      staticFrame_;
  typename octree::JPCCOctreePointCloud<pcl::PointXYZ, octree::OctreeContainerEditableIndex>::Ptr staticOctree_;

 public:
  void set(const JPCCHeader& header);

  void combine(IJPCCCombinationContext& context, bool parallel);

 protected:
  void combineStaticAddedRemoved(const PclFramePtr<pcl::PointXYZ>& staticAddedFrame,
                                 const PclFramePtr<pcl::PointXYZ>& staticRemovedFrame,
                                 FramePtr&                         staticReconstructFrame);

  void combineStaticAddedRemoved(const GroupOfPclFrame<pcl::PointXYZ>& staticAddedFrames,
                                 const GroupOfPclFrame<pcl::PointXYZ>& staticRemovedFrames,
                                 GroupOfFrame&                         staticReconstructFrames);

  void combineDynamicStatic(const FramePtr& dynamicFrame, const FramePtr& staticFrame, FramePtr& reconstructFrame);

  void combineDynamicStatic(const GroupOfFrame& dynamicFrames,
                            const GroupOfFrame& staticFrames,
                            GroupOfFrame&       reconstructFrames,
                            bool                parallel);
};

}  // namespace jpcc::combination
