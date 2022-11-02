#include <limits>

#include <jpcc/segmentation/IOctreeContainerGMM.h>

namespace jpcc::segmentation {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT>
JPCCSegmentationOPCGMMCenter<LeafContainerT>::JPCCSegmentationOPCGMMCenter(const JPCCSegmentationParameter& parameter,
                                                                           const int startFrameNumber) :
    JPCCSegmentation(parameter, startFrameNumber), Base(parameter.resolution), builtVector() {
  static_assert(std::is_base_of_v<IOctreeContainerGMM, LeafContainerT>, "invalid template type");
  for (int i = -1; i >= -(this->parameter_.getK()); i--) {
    alternateCentroids_.push_back(static_cast<float>(i) / MAX_INTENSITY);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT>
bool JPCCSegmentationOPCGMMCenter<LeafContainerT>::isBuilt() const {
  return std::all_of(builtVector.begin(), builtVector.end(), [](bool v) { return v; });
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT>
void JPCCSegmentationOPCGMMCenter<LeafContainerT>::appendTrainSamplesAndBuild(
    const FramePtr& frame, const PclFramePtr<pcl::PointXYZI>& pclFrame) {
  if (this->isBuilt()) { return; }
  this->addFrame(pclFrame);
  this->appendTrainSamples(frame);
  this->build(frame);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT>
void JPCCSegmentationOPCGMMCenter<LeafContainerT>::segmentation(IJPCCSegmentationContext& context, const size_t index) {
  const FrameConstPtr               frame = !context.getFrames().empty() ? context.getFrames()[index] : nullptr;
  const PclFramePtr<pcl::PointXYZI> pclFrame =
      !context.getPclFrames().empty() ? context.getPclFrames()[index] : nullptr;
  const FramePtr dynamicFrame = !context.getDynamicFrames().empty() ? context.getDynamicFrames()[index] : nullptr;
  const FramePtr staticFrame  = !context.getStaticFrames().empty() ? context.getStaticFrames()[index] : nullptr;
  const FramePtr staticAddedFrame =
      !context.getStaticAddedFrames().empty() ? context.getStaticAddedFrames()[index] : nullptr;
  const FramePtr staticRemovedFrame =
      !context.getStaticRemovedFrames().empty() ? context.getStaticRemovedFrames()[index] : nullptr;
  if (dynamicFrame) {
    dynamicFrame->clear();
    dynamicFrame->getFrameNumber() = frame->getFrameNumber();
  }
  if (staticFrame) {
    staticFrame->clear();
    staticFrame->getFrameNumber() = frame->getFrameNumber();
  }
  if (staticAddedFrame) {
    staticAddedFrame->clear();
    staticAddedFrame->getFrameNumber() = frame->getFrameNumber();
  }
  if (staticRemovedFrame) {
    staticRemovedFrame->clear();
    staticRemovedFrame->getFrameNumber() = frame->getFrameNumber();
  }

  this->addFrame(pclFrame);

  OctreeKey key;
  this->segmentationRecursive(frame, dynamicFrame, staticFrame, staticAddedFrame, staticRemovedFrame, key,
                              this->root_node_);

  if (dynamicFrame) {
    cout << "segmentation dynamic "
         << "frameNumber=" << dynamicFrame->getFrameNumber() << ", "
         << "points=" << dynamicFrame->getPointCount() << endl;
  }
  if (staticFrame) {
    cout << "segmentation static "
         << "frameNumber=" << staticFrame->getFrameNumber() << ", "
         << "points=" << staticFrame->getPointCount() << endl;
  }
  if (staticAddedFrame) {
    cout << "segmentation static added "
         << "frameNumber=" << staticAddedFrame->getFrameNumber() << ", "
         << "points=" << staticAddedFrame->getPointCount() << endl;
  }
  if (staticRemovedFrame) {
    cout << "segmentation static removed "
         << "frameNumber=" << staticRemovedFrame->getFrameNumber() << ", "
         << "points=" << staticRemovedFrame->getPointCount() << endl;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT>
void JPCCSegmentationOPCGMMCenter<LeafContainerT>::appendTrainSamples(const FramePtr& frame) {
  if (this->isBuilt()) { return; }
  this->addTrainSampleAndResetLastPointRecursive(frame, this->root_node_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT>
void JPCCSegmentationOPCGMMCenter<LeafContainerT>::addTrainSampleAndResetLastPointRecursive(
    const FramePtr& frame, const BranchNode* branchNode) {
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
template <typename LeafContainerT>
void JPCCSegmentationOPCGMMCenter<LeafContainerT>::build(const FramePtr& frame) {
  if (this->isBuilt()) { return; }
  for (size_t i = 0; i < SIZE; i++) {
    if (!this->builtVector[i] &&
        (frame->getFrameNumber() - this->startFrameNumber_ + 1) >= this->parameter_.getNTrain(i)) {
      this->buildRecursive(frame, i, this->root_node_);
      this->builtVector[i] = true;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT>
void JPCCSegmentationOPCGMMCenter<LeafContainerT>::buildRecursive(const FramePtr&   frame,
                                                                  const size_t      index,
                                                                  const BranchNode* branchNode) {
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
template <typename LeafContainerT>
void JPCCSegmentationOPCGMMCenter<LeafContainerT>::segmentationRecursive(const FrameConstPtr& frame,
                                                                         const FramePtr&      dynamicFrame,
                                                                         const FramePtr&      staticFrame,
                                                                         const FramePtr&      staticAddedFrame,
                                                                         const FramePtr&      staticRemovedFrame,
                                                                         OctreeKey&           key,
                                                                         const BranchNode*    branchNode) {
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

          pcl::PointXYZI& point = leafContainer.getLastPoint();
          if (dynamicFrame && isDynamic) {
            assert(!std::isnan(point.x));
            dynamicFrame->addPoint(Frame::PointType{int32_t(point.x), int32_t(point.y), int32_t(point.z)});
          }

          pcl::PointXYZI center;
          this->genLeafNodeCenterFromOctreeKey(key, center);

          if (staticFrame && isStatic) {
            staticFrame->addPoint(Frame::PointType{int32_t(center.x), int32_t(center.y), int32_t(center.z)});
          }
          if (staticAddedFrame && !lastIsStatic && isStatic) {
            staticAddedFrame->addPoint(Frame::PointType{int32_t(center.x), int32_t(center.y), int32_t(center.z)});
          }
          if (staticRemovedFrame && lastIsStatic && !isStatic) {
            staticRemovedFrame->addPoint(Frame::PointType{int32_t(center.x), int32_t(center.y), int32_t(center.z)});
          }

          leafContainer.setIsLastStatic(isStatic);
          for (size_t i = 0; i < SIZE; i++) {
            if (this->parameter_.updateModelBeforeNTrain ||
                frame->getFrameNumber() >= this->startFrameNumber_ + this->parameter_.getNTrain(i)) {
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