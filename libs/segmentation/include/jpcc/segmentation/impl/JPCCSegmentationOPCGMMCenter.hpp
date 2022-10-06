#include <limits>

#include <jpcc/segmentation/IOctreeContainerGMM.h>

namespace jpcc::segmentation {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename LeafContainerT>
JPCCSegmentationOPCGMMCenter<PointT, LeafContainerT>::JPCCSegmentationOPCGMMCenter(
    const JPCCSegmentationParameter& parameter, const int startFrameNumber) :
    JPCCSegmentation<PointT>(parameter, startFrameNumber), Base(parameter.resolution), builtVector() {
  static_assert(std::is_base_of_v<IOctreeContainerGMM, LeafContainerT>, "invalid template type");
  for (int i = -1; i >= -(this->parameter_.getK()); i--) {
    alternateCentroids_.push_back(static_cast<float>(i) / MAX_INTENSITY);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename LeafContainerT>
bool JPCCSegmentationOPCGMMCenter<PointT, LeafContainerT>::isBuilt() const {
  return std::all_of(builtVector.begin(), builtVector.end(), [](bool v) { return v; });
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename LeafContainerT>
void JPCCSegmentationOPCGMMCenter<PointT, LeafContainerT>::appendTrainSamplesAndBuild(const FramePtr<PointT>& frame) {
  if (this->isBuilt()) { return; }
  this->addFrame(frame);
  this->appendTrainSamples(frame);
  this->build(frame);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename LeafContainerT>
void JPCCSegmentationOPCGMMCenter<PointT, LeafContainerT>::segmentation(IJPCCSegmentationContext<PointT>& context,
                                                                        const size_t                      index) {
  const FrameConstPtr<PointT> frame = !context.getPclFrames().empty() ? context.getPclFrames()[index] : nullptr;
  const FramePtr<PointT>      dynamicFrame =
      !context.getDynamicPclFrames().empty() ? context.getDynamicPclFrames()[index] : nullptr;
  const FramePtr<PointT> staticFrame =
      !context.getStaticPclFrames().empty() ? context.getStaticPclFrames()[index] : nullptr;
  const FramePtr<PointT> staticAddedFrame =
      !context.getStaticAddedPclFrames().empty() ? context.getStaticAddedPclFrames()[index] : nullptr;
  const FramePtr<PointT> staticRemovedFrame =
      !context.getStaticRemovedPclFrames().empty() ? context.getStaticRemovedPclFrames()[index] : nullptr;
  if (dynamicFrame) {
    dynamicFrame->clear();
    dynamicFrame->header = frame->header;
  }
  if (staticFrame) {
    staticFrame->clear();
    staticFrame->header = frame->header;
  }
  if (staticAddedFrame) {
    staticAddedFrame->clear();
    staticAddedFrame->header = frame->header;
  }
  if (staticRemovedFrame) {
    staticRemovedFrame->clear();
    staticRemovedFrame->header = frame->header;
  }

  this->addFrame(frame);

  OctreeKey key;
  this->segmentationRecursive(frame, dynamicFrame, staticFrame, staticAddedFrame, staticRemovedFrame, key,
                              this->root_node_);

  if (dynamicFrame) {
    dynamicFrame->width  = dynamicFrame->size();
    dynamicFrame->height = 1;
    cout << "segmentation dynamic "
         << "frameNumber=" << dynamicFrame->header.seq << ", "
         << "points=" << dynamicFrame->size() << endl;
  }
  if (staticFrame) {
    staticFrame->width  = staticFrame->size();
    staticFrame->height = 1;
    cout << "segmentation static "
         << "frameNumber=" << staticFrame->header.seq << ", "
         << "points=" << staticFrame->size() << endl;
  }
  if (staticAddedFrame) {
    staticAddedFrame->width  = staticAddedFrame->size();
    staticAddedFrame->height = 1;
    cout << "segmentation static added "
         << "frameNumber=" << staticAddedFrame->header.seq << ", "
         << "points=" << staticAddedFrame->size() << endl;
  }
  if (staticRemovedFrame) {
    staticRemovedFrame->width  = staticRemovedFrame->size();
    staticRemovedFrame->height = 1;
    cout << "segmentation static removed "
         << "frameNumber=" << staticRemovedFrame->header.seq << ", "
         << "points=" << staticRemovedFrame->size() << endl;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename LeafContainerT>
void JPCCSegmentationOPCGMMCenter<PointT, LeafContainerT>::appendTrainSamples(const FramePtr<PointT>& frame) {
  if (this->isBuilt()) { return; }
  this->addTrainSampleAndResetLastPointRecursive(frame, this->root_node_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename LeafContainerT>
void JPCCSegmentationOPCGMMCenter<PointT, LeafContainerT>::addTrainSampleAndResetLastPointRecursive(
    const FramePtr<PointT>& frame, const BranchNode* branchNode) {
  for (unsigned char childIndex = 0; childIndex < 8; childIndex++) {
    if (branchNode->hasChild(childIndex)) {
      OctreeNode* childNode = branchNode->getChildPtr(childIndex);
      switch (childNode->getNodeType()) {
        case pcl::octree::BRANCH_NODE: {
          addTrainSampleAndResetLastPointRecursive(frame, dynamic_cast<const BranchNode*>(childNode));
          break;
        }
        case pcl::octree::LEAF_NODE: {
          LeafContainerT& leafContainer = dynamic_cast<LeafNode*>(childNode)->getContainer();
          leafContainer.addTrainSample();
          leafContainer.resetLastPoint();
          break;
        }
        default: break;
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename LeafContainerT>
void JPCCSegmentationOPCGMMCenter<PointT, LeafContainerT>::build(const FramePtr<PointT>& frame) {
  if (this->isBuilt()) { return; }
  for (size_t i = 0; i < SIZE; i++) {
    if (!this->builtVector[i] && (frame->header.seq - this->startFrameNumber_ + 1) >= this->parameter_.getNTrain(i)) {
      this->buildRecursive(frame, i, this->root_node_);
      this->builtVector[i] = true;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename LeafContainerT>
void JPCCSegmentationOPCGMMCenter<PointT, LeafContainerT>::buildRecursive(const FramePtr<PointT>& frame,
                                                                          const size_t            index,
                                                                          const BranchNode*       branchNode) {
  for (unsigned char childIndex = 0; childIndex < 8; childIndex++) {
    if (branchNode->hasChild(childIndex)) {
      OctreeNode* childNode = branchNode->getChildPtr(childIndex);
      switch (childNode->getNodeType()) {
        case pcl::octree::BRANCH_NODE: {
          buildRecursive(frame, index, dynamic_cast<const BranchNode*>(childNode));
          break;
        }
        case pcl::octree::LEAF_NODE: {
          LeafContainerT& leafContainer = dynamic_cast<LeafNode*>(childNode)->getContainer();
          leafContainer.build(index, this->parameter_.getNTrain(index), this->parameter_.getK(index),
                              this->parameter_.getAlpha(index), this->parameter_.minimumVariance, alternateCentroids_);
          break;
        }
        default: break;
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename LeafContainerT>
void JPCCSegmentationOPCGMMCenter<PointT, LeafContainerT>::segmentationRecursive(
    const FrameConstPtr<PointT>& frame,
    const FramePtr<PointT>&      dynamicFrame,
    const FramePtr<PointT>&      staticFrame,
    const FramePtr<PointT>&      staticAddedFrame,
    const FramePtr<PointT>&      staticRemovedFrame,
    OctreeKey&                   key,
    const BranchNode*            branchNode) {
  for (unsigned char childIndex = 0; childIndex < 8; childIndex++) {
    if (branchNode->hasChild(childIndex)) {
      // add current branch voxel to key
      key.pushBranch(childIndex);
      OctreeNode* childNode = branchNode->getChildPtr(childIndex);
      switch (childNode->getNodeType()) {
        case pcl::octree::BRANCH_NODE: {
          segmentationRecursive(frame, dynamicFrame, staticFrame, staticAddedFrame, staticRemovedFrame, key,
                                dynamic_cast<const BranchNode*>(childNode));
          break;
        }
        case pcl::octree::LEAF_NODE: {
          LeafContainerT& leafContainer = dynamic_cast<LeafNode*>(childNode)->getContainer();
          for (size_t i = 0; i < SIZE; i++) {
            if (!leafContainer.isBuilt(i)) {
              leafContainer.build(i, this->parameter_.getNTrain(i), this->parameter_.getK(i),
                                  this->parameter_.getAlpha(i), this->parameter_.minimumVariance, alternateCentroids_);
            }
          }
          bool lastIsStatic = leafContainer.isLastStatic();
          bool isStatic     = leafContainer.isStatic(this->parameter_.getStaticThresholdVector(),
                                                     this->parameter_.getNullStaticThresholdVector(),
                                                     this->parameter_.getOutputExistsPointOnlyVector());
          bool isDynamic    = !isStatic && !std::isnan(leafContainer.getLastPoint().intensity);

          PointT& point = leafContainer.getLastPoint();
          if (dynamicFrame && isDynamic) {
            assert(!std::isnan(point.x));
            dynamicFrame->points.push_back(point);
          }

          PointT center;
          this->genLeafNodeCenterFromOctreeKey(key, center);
          if constexpr (pcl::traits::has_intensity_v<PointT>) { center.intensity = point.intensity; }
          if constexpr (pcl::traits::has_normal_v<PointT>) {
            center.normal_x  = point.normal_x;
            center.normal_y  = point.normal_y;
            center.normal_z  = point.normal_z;
            center.curvature = point.curvature;
          }

          if (staticFrame && isStatic) { staticFrame->points.push_back(center); }
          if (staticAddedFrame && !lastIsStatic && isStatic) { staticAddedFrame->points.push_back(center); }
          if (staticRemovedFrame && lastIsStatic && !isStatic) { staticRemovedFrame->points.push_back(center); }

          leafContainer.setIsLastStatic(isStatic);
          for (size_t i = 0; i < SIZE; i++) {
            if (this->parameter_.updateModelBeforeNTrain ||
                frame->header.seq >= this->startFrameNumber_ + this->parameter_.getNTrain(i)) {
              leafContainer.updateModel(i, this->parameter_.getAlpha(i),
                                        this->parameter_.getAlpha(i) * this->parameter_.getNullAlphaRatio(i),
                                        this->parameter_.minimumVariance);
            }
          }
          leafContainer.resetLastPoint();
          break;
        }
        default: break;
      }
      // pop current branch voxel from key
      key.popBranch();
    }
  }
}

}  // namespace jpcc::segmentation