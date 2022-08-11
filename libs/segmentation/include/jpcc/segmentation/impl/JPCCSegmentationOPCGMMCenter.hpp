#include <limits>

namespace jpcc::segmentation {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCSegmentationOPCGMMCenter<PointT>::JPCCSegmentationOPCGMMCenter(const JPCCSegmentationParameter& parameter,
                                                                   const int                        startFrameNumber) :
    JPCCSegmentation<PointT>(parameter, startFrameNumber), Base(parameter.resolution) {
  for (int i = -1; i >= -(this->parameter_.getK()); i--) {
    alternateCentroids_.push_back(static_cast<float>(i) / MAX_INTENSITY);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCSegmentationOPCGMMCenter<PointT>::appendTrainSamples(FramePtr<PointT> frame) {
  if (this->built_) { return; }
  for (auto it = this->leaf_depth_begin(), end = this->leaf_depth_end(); it != end; ++it) {
    it.getLeafContainer().resetLastPoint();
  }
  this->addFrame(frame);
  for (auto it = this->leaf_depth_begin(), end = this->leaf_depth_end(); it != end; ++it) {
    it.getLeafContainer().addTrainSample();
  }
  if ((frame->header.seq - this->startFrameNumber_ + 1) >= this->parameter_.getNTrain()) {
    for (auto it = this->leaf_depth_begin(), end = this->leaf_depth_end(); it != end; ++it) {
      LeafContainer& leafContainer = it.getLeafContainer();
      assert(!leafContainer.isBuilt());
      leafContainer.build(this->parameter_.getNTrain(), this->parameter_.getK(), this->parameter_.getAlpha(),
                          this->parameter_.minimumVariance, alternateCentroids_);
    }
    this->built_ = true;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCSegmentationOPCGMMCenter<PointT>::segmentation(const FrameConstPtr<PointT>& frame,
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
    if (!leafContainer.isBuilt()) {
      leafContainer.build(this->parameter_.getNTrain(), this->parameter_.getK(), this->parameter_.getAlpha(),
                          this->parameter_.minimumVariance, alternateCentroids_);
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
    if (this->parameter_.updateModelBeforeNTrain ||
        frame->header.seq >= this->startFrameNumber_ + this->parameter_.getNTrain()) {
      leafContainer.updateModel(this->parameter_.getAlpha(),
                                this->parameter_.getAlpha() * this->parameter_.getNullAlphaRatio(),
                                this->parameter_.minimumVariance);
    }
  }
}

}  // namespace jpcc::segmentation