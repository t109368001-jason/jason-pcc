#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/octree/OctreeContainerEditableIndex.h>

namespace jpcc::segmentation {

template <typename PointT>
class JPCCCombination {
 protected:
  FramePtr<PointT>                                                                         staticFrame_;
  typename octree::JPCCOctreePointCloud<PointT, octree::OctreeContainerEditableIndex>::Ptr staticOctree_;

 public:
  JPCCCombination(double resolution);

  void combine(IJPCCCombinationContext<PointT>& context, bool parallel);

 protected:
  void combine(const FramePtr<PointT>& dynamicFrame,
               const FramePtr<PointT>& staticFrame,
               const FramePtr<PointT>& staticAddedFrame,
               const FramePtr<PointT>& staticRemovedFrame,
               FramePtr<PointT>&       staticReconstructFrame,
               FramePtr<PointT>&       reconstructFrame);

  void combineStaticAddedRemoved(const FramePtr<PointT>& staticAddedFrame,
                                 const FramePtr<PointT>& staticRemovedFrame,
                                 FramePtr<PointT>&       staticReconstructFrame);

  void combineStaticAddedRemoved(const GroupOfFrame<PointT>& staticAddedFrames,
                                 const GroupOfFrame<PointT>& staticRemovedFrames,
                                 GroupOfFrame<PointT>&       staticReconstructFrames);

  void combineDynamicStatic(const FramePtr<PointT>& dynamicFrame,
                            const FramePtr<PointT>& staticFrame,
                            FramePtr<PointT>&       reconstructFrame);

  void combineDynamicStatic(const GroupOfFrame<PointT>& dynamicFrames,
                            const GroupOfFrame<PointT>& staticFrames,
                            GroupOfFrame<PointT>&       reconstructFrames,
                            bool                        parallel);
};

}  // namespace jpcc::segmentation

#include <jpcc/segmentation/impl/JPCCCombination.hpp>
