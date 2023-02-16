#include <limits>

#include <boost/log/trivial.hpp>

#include <jpcc/segmentation/IOctreeContainerGMM.h>

namespace jpcc::segmentation {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT>
JPCCSegmentationOPCGMMCenter<LeafContainerT>::JPCCSegmentationOPCGMMCenter(const JPCCSegmentationParameter& parameter,
                                                                           const int startFrameNumber) :
    JPCCSegmentation(parameter, startFrameNumber), Base(parameter.resolution), builtVector() {
  static_assert(std::is_base_of_v<IOctreeContainerGMM, LeafContainerT>, "invalid template type");
  for (Intensity i = MAX_INTENSITY + 1; i <= (this->parameter_.getK() + MAX_INTENSITY + 1); i++) {
    alternateCentroids_.insert(i);
  }
  THROW_IF_NOT(alternateCentroids_.size() >= this->parameter_.getK());
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT>
bool JPCCSegmentationOPCGMMCenter<LeafContainerT>::isBuilt() const {
  return std::all_of(builtVector.begin(), builtVector.end(), [](bool v) { return v; });
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT>
void JPCCSegmentationOPCGMMCenter<LeafContainerT>::appendTrainSamplesAndBuild(const FramePtr&            frame,
                                                                              const PclFramePtr<PointT>& pclFrame) {
  if (this->isBuilt()) { return; }
  this->addFrame(pclFrame);
  this->appendTrainSamples(frame);
  this->build(frame);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename LeafContainerT>
void JPCCSegmentationOPCGMMCenter<LeafContainerT>::segmentation(IJPCCSegmentationContext& context, const size_t index) {
  const FrameConstPtr       frame        = context.getFrames()[index];
  const PclFramePtr<PointT> pclFrame     = context.getPclFrames()[index];
  const FramePtr            dynamicFrame = context.getDynamicFrames()[index];
  const FramePtr            staticFrame  = context.getSegmentationOutputType() == SegmentationOutputType::DYNAMIC_STATIC
                                               ? context.getStaticFrames()[index]
                                               : nullptr;
  const FramePtr            staticAddedFrame =
      context.getSegmentationOutputType() == SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED
                     ? context.getStaticAddedFrames()[index]
                     : nullptr;
  const FramePtr staticRemovedFrame =
      context.getSegmentationOutputType() == SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED
          ? context.getStaticRemovedFrames()[index]
          : nullptr;
  if (dynamicFrame) {
    dynamicFrame->clear();
    dynamicFrame->setFrameNumber(frame->getFrameNumber());
  }
  if (staticFrame) {
    staticFrame->clear();
    staticFrame->setFrameNumber(frame->getFrameNumber());
  }
  if (staticAddedFrame) {
    staticAddedFrame->clear();
    staticAddedFrame->setFrameNumber(frame->getFrameNumber());
  }
  if (staticRemovedFrame) {
    staticRemovedFrame->clear();
    staticRemovedFrame->setFrameNumber(frame->getFrameNumber());
  }

  this->addFrame(pclFrame);

  OctreeKey key;
  this->segmentationRecursive(frame, dynamicFrame, staticFrame, staticAddedFrame, staticRemovedFrame, key,
                              this->root_node_);

  if (dynamicFrame) {
    BOOST_LOG_TRIVIAL(info) << "segmentation dynamic "
                            << "frameNumber_=" << dynamicFrame->getFrameNumber() << ", "
                            << "points=" << dynamicFrame->getPointCount();
  }
  if (staticFrame) {
    BOOST_LOG_TRIVIAL(info) << "segmentation static "
                            << "frameNumber_=" << staticFrame->getFrameNumber() << ", "
                            << "points=" << staticFrame->getPointCount();
  }
  if (staticAddedFrame) {
    BOOST_LOG_TRIVIAL(info) << "segmentation static added "
                            << "frameNumber_=" << staticAddedFrame->getFrameNumber() << ", "
                            << "points=" << staticAddedFrame->getPointCount();
  }
  if (staticRemovedFrame) {
    BOOST_LOG_TRIVIAL(info) << "segmentation static removed "
                            << "frameNumber_=" << staticRemovedFrame->getFrameNumber() << ", "
                            << "points=" << staticRemovedFrame->getPointCount();
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
          bool isStatic     = leafContainer.isStatic(
              this->parameter_.getStaticThreshold1Vector(), this->parameter_.getStaticThreshold2Vector(),
              this->parameter_.getNullStaticThreshold1Vector(), this->parameter_.getNullStaticThreshold2Vector(),
              this->parameter_.getOutputExistsPointOnlyVector(), lastIsStatic);
          bool isDynamic = !isStatic && !std::isnan(leafContainer.getLastPoint().x);

          PointT& point = leafContainer.getLastPoint();
          if (dynamicFrame && isDynamic) {
            assert(!std::isnan(point.x));
            dynamicFrame->addPositionNormal(
                PointType{PointValueType(point.x), PointValueType(point.y), PointValueType(point.z)},
                NormalType{point.normal_x, point.normal_y, point.normal_z});
          }

          PointT center;
          this->genLeafNodeCenterFromOctreeKey(key, center);

          if (staticFrame && isStatic) {
            staticFrame->addPositionNormal(
                PointType{PointValueType(center.x), PointValueType(center.y), PointValueType(center.z)},
                NormalType{point.normal_x, point.normal_y, point.normal_z});
          }
          if (staticAddedFrame && !lastIsStatic && isStatic) {
            staticAddedFrame->addPositionNormal(
                PointType{PointValueType(center.x), PointValueType(center.y), PointValueType(center.z)},
                NormalType{point.normal_x, point.normal_y, point.normal_z});
          }
          if (staticRemovedFrame && lastIsStatic && !isStatic) {
            staticRemovedFrame->addPositionNormal(
                PointType{PointValueType(center.x), PointValueType(center.y), PointValueType(center.z)},
                NormalType{point.normal_x, point.normal_y, point.normal_z});
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