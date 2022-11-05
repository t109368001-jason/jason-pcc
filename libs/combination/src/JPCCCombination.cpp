#include <jpcc/combination/JPCCCombination.h>

#include <execution>

#include <boost/range/counting_range.hpp>

namespace jpcc::combination {

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCCombination::set(const JPCCHeader& header) {
  staticFrame_ = jpcc::make_shared<PclFrame<PointT>>();
  staticOctree_ =
      jpcc::make_shared<octree::JPCCOctreePointCloud<PointT, octree::OctreeContainerEditableIndex>>(header.resolution);
  staticOctree_->setInputCloud(staticFrame_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCCombination::combine(IJPCCCombinationContext& context, bool parallel) {
  context.getReconstructFrames().clear();
  context.getReconstructFrames().resize(context.getDynamicReconstructFrames().size());
  GroupOfFrame _staticFrames = context.getStaticReconstructFrames();
  if (context.getSegmentationOutputType() == SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED) {
    context.getStaticReconstructFrames().resize(context.getDynamicReconstructFrames().size());
    this->combineStaticAddedRemoved(context.getStaticAddedReconstructPclFrames(),
                                    context.getStaticRemovedReconstructPclFrames(),
                                    context.getStaticReconstructFrames());
    _staticFrames = context.getStaticReconstructFrames();
  }
  JPCCCombination::combineDynamicStatic(context.getDynamicReconstructFrames(), _staticFrames,
                                        context.getReconstructFrames(), parallel);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCCombination::combineStaticAddedRemoved(const PclFramePtr<PointT>& staticAddedFrame,
                                                const PclFramePtr<PointT>& staticRemovedFrame,
                                                FramePtr&                  staticReconstructFrame) {
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
  staticReconstructFrame = jpcc::make_shared<Frame>();
  staticReconstructFrame->fromPcl<PointT>(staticFrame_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCCombination::combineStaticAddedRemoved(const GroupOfPclFrame<PointT>& staticAddedFrames,
                                                const GroupOfPclFrame<PointT>& staticRemovedFrames,
                                                GroupOfFrame&                  staticReconstructFrames) {
  staticReconstructFrames.resize(staticAddedFrames.size());
  for (size_t i = 0; i < staticRemovedFrames.size(); i++) {
    this->combineStaticAddedRemoved(staticAddedFrames[i], staticRemovedFrames[i], staticReconstructFrames[i]);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCCombination::combineDynamicStatic(const FramePtr& dynamicFrame,
                                           const FramePtr& staticFrame,
                                           FramePtr&       reconstructFrame) {
  reconstructFrame = jpcc::make_shared<Frame>();
  if (dynamicFrame) { reconstructFrame->append(*dynamicFrame); }
  if (staticFrame) { reconstructFrame->append(*staticFrame); }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCCombination::combineDynamicStatic(const GroupOfFrame& dynamicFrames,
                                           const GroupOfFrame& staticFrames,
                                           GroupOfFrame&       reconstructFrames,
                                           bool                parallel) {
  FramePtr emptyFrame;
  if (!parallel) {
    for (size_t i = 0; i < dynamicFrames.size(); i++) {
      JPCCCombination::combineDynamicStatic(dynamicFrames[i], !staticFrames.empty() ? staticFrames[i] : emptyFrame,
                                            reconstructFrames[i]);
    }
  } else {
    const auto range = boost::counting_range<size_t>(0, dynamicFrames.size());
    std::for_each(std::execution::par, range.begin(), range.end(),
                  [&](const size_t& i) {  //
                    JPCCCombination::combineDynamicStatic(
                        dynamicFrames[i], !staticFrames.empty() ? staticFrames[i] : emptyFrame, reconstructFrames[i]);
                  });
  }
}

}  // namespace jpcc::combination