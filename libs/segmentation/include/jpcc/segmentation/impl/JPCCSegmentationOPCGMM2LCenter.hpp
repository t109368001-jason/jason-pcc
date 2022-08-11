#include <limits>

namespace jpcc::segmentation {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCSegmentationOPCGMM2LCenter<PointT>::JPCCSegmentationOPCGMM2LCenter(const JPCCSegmentationParameter& parameter,
                                                                       const int startFrameNumber) :
    JPCCSegmentation<PointT>(parameter, startFrameNumber), Base(parameter.resolution), builtVector() {
  for (int i = -1; i >= -(this->parameter_.getK()); i--) {
    alternateCentroids_.push_back(static_cast<float>(i) / MAX_INTENSITY);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool JPCCSegmentationOPCGMM2LCenter<PointT>::isBuilt() const {
  return std::all_of(builtVector.begin(), builtVector.end(), [](bool v) { return v; });
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCSegmentationOPCGMM2LCenter<PointT>::appendTrainSamples(FramePtr<PointT> frame) {
  if (this->isBuilt()) { return; }
  for (auto it = this->leaf_depth_begin(), end = this->leaf_depth_end(); it != end; ++it) {
    it.getLeafContainer().resetLastPoint();
  }
  this->addFrame(frame);
  for (auto it = this->leaf_depth_begin(), end = this->leaf_depth_end(); it != end; ++it) {
    it.getLeafContainer().addTrainSample();
  }
  for (size_t i = 0; i < SIZE; i++) {
    if ((frame->header.seq - this->startFrameNumber_ + 1) >= this->parameter_.getNTrain(i)) {
      for (auto it = this->leaf_depth_begin(), end = this->leaf_depth_end(); it != end; ++it) {
        LeafContainer& leafContainer = it.getLeafContainer();
        assert(!leafContainer.isBuilt(i));
        leafContainer.build(i, this->parameter_.getNTrain(i), this->parameter_.getK(i), this->parameter_.getAlpha(i),
                            this->parameter_.minimumVariance, alternateCentroids_);
      }
      this->builtVector.at(i) = true;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCSegmentationOPCGMM2LCenter<PointT>::segmentation(const FrameConstPtr<PointT>& frame,
                                                          FramePtr<PointT>             dynamicFrame,
                                                          FramePtr<PointT>             staticFrame,
                                                          FramePtr<PointT>             staticFrameAdded,
                                                          FramePtr<PointT>             staticFrameRemoved) {
  for (auto it = this->leaf_depth_begin(), end = this->leaf_depth_end(); it != end; ++it) {
    it.getLeafContainer().resetLastPoint();
  }
  this->addFrame(frame);
  for (auto it = this->leaf_depth_begin(), end = this->leaf_depth_end(); it != end; ++it) {
    LeafContainer& leafContainer = it.getLeafContainer();
    for (size_t i = 0; i < SIZE; i++) {
      if (!leafContainer.isBuilt(i)) {
        leafContainer.build(i, this->parameter_.getNTrain(i), this->parameter_.getK(i), this->parameter_.getAlpha(i),
                            this->parameter_.minimumVariance, alternateCentroids_);
      }
    }
    bool lastIsStatic = leafContainer.isLastStatic();
    bool isStatic     = leafContainer.isStatic(this->parameter_.getStaticThresholdVector(),
                                               this->parameter_.getNullStaticThresholdVector());
    bool isDynamic    = isStatic && !std::isnan(leafContainer.getLastPoint().intensity);
    if (dynamicFrame && isDynamic) {
      PointT& point = leafContainer.getLastPoint();
      assert(!std::isnan(point.x));
      dynamicFrame->points.push_back(point);
    }

    PointT center;
    this->genLeafNodeCenterFromOctreeKey(it.getCurrentOctreeKey(), center);

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
  }
}

}  // namespace jpcc::segmentation