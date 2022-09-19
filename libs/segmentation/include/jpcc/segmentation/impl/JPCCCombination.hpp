
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
void JPCCCombination<PointT>::combine(const FramePtr<PointT>& dynamicFrame,
                                      const FramePtr<PointT>& staticFrame,
                                      const FramePtr<PointT>& staticAddedFrame,
                                      const FramePtr<PointT>& staticRemovedFrame,
                                      FramePtr<PointT>&       staticReconstructFrame,
                                      FramePtr<PointT>&       reconstructFrame) {
  FramePtr<PointT> _staticFrame = staticFrame;
  if (!staticFrame && staticAddedFrame && staticRemovedFrame) {
    this->combineStaticAddedRemoved(staticAddedFrame, staticRemovedFrame, staticReconstructFrame);
    _staticFrame = staticReconstructFrame;
  }

  this->combineDynamicStatic(dynamicFrame, _staticFrame, reconstructFrame);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCCombination<PointT>::combine(const GroupOfFrame<PointT>& dynamicFrames,
                                      const GroupOfFrame<PointT>& staticFrames,
                                      const GroupOfFrame<PointT>& staticAddedFrames,
                                      const GroupOfFrame<PointT>& staticRemovedFrames,
                                      GroupOfFrame<PointT>&       staticReconstructFrames,
                                      GroupOfFrame<PointT>&       reconstructFrames,
                                      bool                        parallel) {
  GroupOfFrame<PointT> _staticFrames = staticFrames;
  if (staticFrames.empty() && !staticAddedFrames.empty() && !staticRemovedFrames.empty()) {
    this->combineStaticAddedRemoved(staticAddedFrames, staticRemovedFrames, staticReconstructFrames);
    _staticFrames = staticReconstructFrames;
  }
  this->combineDynamicStatic(dynamicFrames, _staticFrames, reconstructFrames, parallel);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCCombination<PointT>::combineStaticAddedRemoved(const FramePtr<PointT>& staticAddedFrame,
                                                        const FramePtr<PointT>& staticRemovedFrame,
                                                        FramePtr<PointT>&       staticReconstructFrame) {
  if (staticRemovedFrame) {
    for (const PointT& pointToRemove : staticRemovedFrame->points) {
      staticOctree_->deletePointFromCloud(pointToRemove, staticFrame_);
    }
  }
  if (staticAddedFrame) {
    for (const PointT& pointToAdd : staticAddedFrame->points) {
      staticOctree_->addPointToCloud(pointToAdd, staticFrame_);
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
    this->combineStaticAddedRemoved(staticAddedFrames.at(i), staticRemovedFrames.at(i), staticReconstructFrames.at(i));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCCombination<PointT>::combineDynamicStatic(const FramePtr<PointT>& dynamicFrame,
                                                   const FramePtr<PointT>& staticFrame,
                                                   FramePtr<PointT>&       reconstructFrame) {
  pcl::copyPointCloud(*dynamicFrame, *reconstructFrame);
  pcl::copyPointCloud(*staticFrame, *reconstructFrame);
}
//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCCombination<PointT>::combineDynamicStatic(const GroupOfFrame<PointT>& dynamicFrames,
                                                   const GroupOfFrame<PointT>& staticFrames,
                                                   GroupOfFrame<PointT>&       reconstructFrames,
                                                   bool                        parallel) {
  if (!parallel) {
    for (size_t i = 0; i < dynamicFrames.size(); i++) {
      this->combineDynamicStatic(dynamicFrames.at(i), staticFrames.at(i), reconstructFrames.at(i));
    }
  } else {
    const auto range = boost::counting_range<size_t>(0, dynamicFrames.size());
    std::for_each(std::execution::par, range.begin(), range.end(),
                  [&](const size_t& i) {  //
                    this->combineDynamicStatic(dynamicFrames.at(i), staticFrames.at(i), reconstructFrames.at(i));
                  });
  }
}

}  // namespace jpcc::segmentation