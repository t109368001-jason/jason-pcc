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
void JPCCSegmentationOPCGMMCenter<PointT, LeafContainerT>::appendTrainSamples(FramePtr<PointT> frame) {
  if (this->isBuilt()) { return; }
  for (auto it = this->leaf_depth_begin(), end = this->leaf_depth_end(); it != end; ++it) {
    it.getLeafContainer().resetLastPoint();
  }
  this->addFrame(frame);
  for (auto it = this->leaf_depth_begin(), end = this->leaf_depth_end(); it != end; ++it) {
    it.getLeafContainer().addTrainSample();
  }
  for (size_t i = 0; i < SIZE; i++) {
    if (!this->builtVector.at(i) &&
        (frame->header.seq - this->startFrameNumber_ + 1) >= this->parameter_.getNTrain(i)) {
      for (auto it = this->leaf_depth_begin(), end = this->leaf_depth_end(); it != end; ++it) {
        LeafContainerT& leafContainer = it.getLeafContainer();
        leafContainer.build(i, this->parameter_.getNTrain(i), this->parameter_.getK(i), this->parameter_.getAlpha(i),
                            this->parameter_.minimumVariance, alternateCentroids_);
      }
      this->builtVector.at(i) = true;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename LeafContainerT>
void JPCCSegmentationOPCGMMCenter<PointT, LeafContainerT>::segmentation(const FrameConstPtr<PointT>& frame,
                                                                        FramePtr<PointT>             dynamicFrame,
                                                                        FramePtr<PointT>             staticFrame,
                                                                        FramePtr<PointT>             staticFrameAdded,
                                                                        FramePtr<PointT> staticFrameRemoved) {
  for (auto it = this->leaf_depth_begin(), end = this->leaf_depth_end(); it != end; ++it) {
    it.getLeafContainer().resetLastPoint();
  }
  this->addFrame(frame);
  for (auto it = this->leaf_depth_begin(), end = this->leaf_depth_end(); it != end; ++it) {
    LeafContainerT& leafContainer = it.getLeafContainer();
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