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
void JPCCSegmentationOPCGMMCenter<PointT, LeafContainerT>::segmentation(const FrameConstPtr<PointT>& frame,
                                                                        const FramePtr<PointT>&      dynamicFrame,
                                                                        const FramePtr<PointT>&      staticFrame,
                                                                        const FramePtr<PointT>&      staticFrameAdded,
                                                                        const FramePtr<PointT>& staticFrameRemoved) {
  if (dynamicFrame) {
    dynamicFrame->clear();
    dynamicFrame->header = frame->header;
  }
  if (staticFrame) {
    staticFrame->clear();
    staticFrame->header = frame->header;
  }
  if (staticFrameAdded) {
    staticFrameAdded->clear();
    staticFrameAdded->header = frame->header;
  }
  if (staticFrameRemoved) {
    staticFrameRemoved->clear();
    staticFrameRemoved->header = frame->header;
  }

  this->addFrame(frame);

  OctreeKey key;
  this->segmentationRecursive(frame, dynamicFrame, staticFrame, staticFrameAdded, staticFrameRemoved, key,
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
  if (staticFrameAdded) {
    staticFrameAdded->width  = staticFrameAdded->size();
    staticFrameAdded->height = 1;
    cout << "segmentation static added "
         << "frameNumber=" << staticFrameAdded->header.seq << ", "
         << "points=" << staticFrameAdded->size() << endl;
  }
  if (staticFrameRemoved) {
    staticFrameRemoved->width  = staticFrameRemoved->size();
    staticFrameRemoved->height = 1;
    cout << "segmentation static removed "
         << "frameNumber=" << staticFrameRemoved->header.seq << ", "
         << "points=" << staticFrameRemoved->size() << endl;
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
    if (!this->builtVector.at(i) &&
        (frame->header.seq - this->startFrameNumber_ + 1) >= this->parameter_.getNTrain(i)) {
      this->buildRecursive(frame, i, this->root_node_);
      this->builtVector.at(i) = true;
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
    const FramePtr<PointT>&      staticFrameAdded,
    const FramePtr<PointT>&      staticFrameRemoved,
    OctreeKey&                   key,
    const BranchNode*            branchNode) {
  for (unsigned char childIndex = 0; childIndex < 8; childIndex++) {
    if (branchNode->hasChild(childIndex)) {
      // add current branch voxel to key
      key.pushBranch(childIndex);
      OctreeNode* childNode = branchNode->getChildPtr(childIndex);
      switch (childNode->getNodeType()) {
        case pcl::octree::BRANCH_NODE: {
          segmentationRecursive(frame, dynamicFrame, staticFrame, staticFrameAdded, staticFrameRemoved, key,
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
          if (dynamicFrame && isDynamic) {
            PointT& point = leafContainer.getLastPoint();
            assert(!std::isnan(point.x));
            dynamicFrame->points.push_back(point);
          }

          PointT center;
          this->genLeafNodeCenterFromOctreeKey(key, center);

          if (staticFrame && isStatic) { staticFrame->points.push_back(center); }
          if (staticFrameAdded && !lastIsStatic && isStatic) { staticFrameAdded->points.push_back(center); }
          if (staticFrameRemoved && lastIsStatic && !isStatic) { staticFrameRemoved->points.push_back(center); }

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