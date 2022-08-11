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
    bool isLastStatic = leafContainer.isLastStatic();

    //    if (isLastStatic && staticFrame) { staticFrame->push_back(leafContainer.getPoint()); }
    if (!std::isnan(leafContainer.getLastPoint().intensity)) {
      const float intensity = leafContainer.getLastPoint().intensity;

      if (dynamicFrame) {
        double probability = leafContainer.getProbability(intensity);

        bool isDynamic = probability < this->parameter_.getDynamicThresholdLE();

        if (!isLastStatic && isDynamic) {
          PointT& point = leafContainer.getLastPoint();
          assert(!std::isnan(point.x));
          dynamicFrame->points.push_back(point);
        }
      }
    }
    if (this->parameter_.updateModelBeforeNTrain ||
        frame->header.seq >= this->startFrameNumber_ + this->parameter_.getNTrain()) {
      leafContainer.updateModel(this->parameter_.getAlpha(),
                                this->parameter_.getAlpha() * this->parameter_.getNullAlphaRatio(),
                                this->parameter_.minimumVariance);
    }
    bool isStatic = leafContainer.getStaticProbability() > this->parameter_.getStaticThresholdGT();
    if (this->parameter_.getOutputExistsPointOnly()) {
      isStatic &= !std::isnan(leafContainer.getLastPoint().intensity);
    }
    PointT center;
    this->genLeafNodeCenterFromOctreeKey(it.getCurrentOctreeKey(), center);

    if (staticFrame && isStatic) { staticFrame->points.push_back(center); }
    if (staticFrameAdded && !isLastStatic && isStatic) { staticFrameAdded->points.push_back(center); }
    if (staticFrameRemoved && isLastStatic && !isStatic) { staticFrameRemoved->points.push_back(center); }

    leafContainer.setIsLastStatic(isStatic);
  }
}

}  // namespace jpcc::segmentation