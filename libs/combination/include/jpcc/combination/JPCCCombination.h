#pragma once

#include <jpcc/common/JPCCHeader.h>
#include <jpcc/common/Common.h>
#include <jpcc/common/IJPCCCombinationContext.h>
#include <jpcc/octree/JPCCOctreePointCloud.h>
#include <jpcc/octree/OctreeContainerEditableIndex.h>

namespace jpcc::combination {

class JPCCCombination {
 public:
  using PointT = PointCombination;

 protected:
  PclFramePtr<PointT>                                                                      staticFrame_;
  typename octree::JPCCOctreePointCloud<PointT, octree::OctreeContainerEditableIndex>::Ptr staticOctree_;

 public:
  void set(const JPCCHeader& header);

  void combine(IJPCCCombinationContext& context, bool parallel);

 protected:
  void combineStaticAddedRemoved(const PclFramePtr<PointT>& staticAddedFrame,
                                 const PclFramePtr<PointT>& staticRemovedFrame,
                                 FramePtr&                  staticReconstructFrame);

  void combineStaticAddedRemoved(const GroupOfPclFrame<PointT>& staticAddedFrames,
                                 const GroupOfPclFrame<PointT>& staticRemovedFrames,
                                 GroupOfFrame&                  staticReconstructFrames);

  static void combineDynamicStatic(const FramePtr& dynamicFrame,
                                   const FramePtr& staticFrame,
                                   FramePtr&       reconstructFrame);

  static void combineDynamicStatic(const GroupOfFrame& dynamicFrames,
                                   const GroupOfFrame& staticFrames,
                                   GroupOfFrame&       reconstructFrames,
                                   bool                parallel);
};

}  // namespace jpcc::combination
