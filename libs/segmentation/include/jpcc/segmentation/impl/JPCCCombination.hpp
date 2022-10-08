
#include "jpcc/segmentation/JPCCCombination.h"

namespace jpcc::segmentation {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCCombination<PointT>::JPCCCombination(const double resolution) {
  staticFrame_ = jpcc::make_shared<Frame<PointT>>();
  staticOctree_ =
      jpcc::make_shared<octree::JPCCOctreePointCloud<PointT, octree::OctreeContainerEditableIndex>>(resolution);
  staticOctree_->setInputCloud(staticFrame_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCCombination<PointT>::combine(IJPCCCombinationContext<PointT>& context, bool parallel) {
  context.getReconstructPclFrames().clear();
  context.getReconstructPclFrames().resize(context.getDynamicReconstructPclFrames().size());
  GroupOfFrame<PointT> _staticFrames = context.getStaticReconstructPclFrames();
  if (context.getSegmentationOutputType() == SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED) {
    context.getStaticReconstructPclFrames().resize(context.getDynamicReconstructPclFrames().size());
    this->combineStaticAddedRemoved(context.getStaticAddedReconstructPclFrames(),
                                    context.getStaticRemovedReconstructPclFrames(),
                                    context.getStaticReconstructPclFrames());
    _staticFrames = context.getStaticReconstructPclFrames();
  }
  this->combineDynamicStatic(context.getDynamicReconstructPclFrames(), _staticFrames, context.getReconstructPclFrames(),
                             parallel);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCCombination<PointT>::combineStaticAddedRemoved(const FramePtr<PointT>& staticAddedFrame,
                                                        const FramePtr<PointT>& staticRemovedFrame,
                                                        FramePtr<PointT>&       staticReconstructFrame) {
#if !defined(NDEBUG)
  size_t expectedSize = staticFrame_->size();
#endif
  if (staticRemovedFrame) {
    for (const PointT& pointToRemove : staticRemovedFrame->points) {
      staticOctree_->deletePointFromCloud(pointToRemove, staticFrame_);
#if !defined(NDEBUG)
      expectedSize--;
      assert(staticFrame_->size() == expectedSize);
#endif
    }
  }
  if (staticAddedFrame) {
    for (const PointT& pointToAdd : staticAddedFrame->points) {
      staticOctree_->addPointToCloud(pointToAdd, staticFrame_);
#if !defined(NDEBUG)
      expectedSize++;
      assert(staticFrame_->size() == expectedSize);
#endif
    }
  }
  staticReconstructFrame = jpcc::make_shared<Frame<PointT>>();
  pcl::copyPointCloud(*staticFrame_, *staticReconstructFrame);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCCombination<PointT>::combineStaticAddedRemoved(const GroupOfFrame<PointT>& staticAddedFrames,
                                                        const GroupOfFrame<PointT>& staticRemovedFrames,
                                                        GroupOfFrame<PointT>&       staticReconstructFrames) {
  staticReconstructFrames.resize(staticAddedFrames.size());
  for (size_t i = 0; i < staticRemovedFrames.size(); i++) {
    this->combineStaticAddedRemoved(staticAddedFrames[i], staticRemovedFrames[i], staticReconstructFrames[i]);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCCombination<PointT>::combineDynamicStatic(const FramePtr<PointT>& dynamicFrame,
                                                   const FramePtr<PointT>& staticFrame,
                                                   FramePtr<PointT>&       reconstructFrame) {
  reconstructFrame = jpcc::make_shared<Frame<PointT>>();
  if (dynamicFrame) { *reconstructFrame += *dynamicFrame; }
  if (staticFrame) { *reconstructFrame += *staticFrame; }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCCombination<PointT>::combineDynamicStatic(const GroupOfFrame<PointT>& dynamicFrames,
                                                   const GroupOfFrame<PointT>& staticFrames,
                                                   GroupOfFrame<PointT>&       reconstructFrames,
                                                   bool                        parallel) {
  FramePtr<PointT> emptyFrame;
  if (!parallel) {
    for (size_t i = 0; i < dynamicFrames.size(); i++) {
      this->combineDynamicStatic(dynamicFrames[i], !staticFrames.empty() ? staticFrames[i] : emptyFrame,
                                 reconstructFrames[i]);
    }
  } else {
    const auto range = boost::counting_range<size_t>(0, dynamicFrames.size());
    std::for_each(std::execution::par, range.begin(), range.end(),
                  [&](const size_t& i) {  //
                    this->combineDynamicStatic(dynamicFrames[i], !staticFrames.empty() ? staticFrames[i] : emptyFrame,
                                               reconstructFrames[i]);
                  });
  }
}

}  // namespace jpcc::segmentation