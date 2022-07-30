#include <limits>

using namespace std;

namespace jpcc::segmentation {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCSegmentationOPCGMMCenter<PointT>::JPCCSegmentationOPCGMMCenter(const JPCCSegmentationParameter& parameter) :
    JPCCSegmentationBase<PointT>(parameter), Base(parameter.resolution) {
  for (int i = -1; i >= -(this->parameter_.k); i--) {
    alternateCentroids_.push_back(static_cast<float>(i) / MAX_INTENSITY);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCSegmentationOPCGMMCenter<PointT>::appendTrainSamples(FramePtr<PointT> frame) {
  for (auto it = this->leaf_depth_begin(), end = this->leaf_depth_end(); it != end; ++it) {
    it.getLeafContainer().resetLastPoint();
  }
  addFrame(frame);
  for (auto it = this->leaf_depth_begin(), end = this->leaf_depth_end(); it != end; ++it) {
    it.getLeafContainer().addTrainSample();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCSegmentationOPCGMMCenter<PointT>::build() {
  for (auto it = this->leaf_depth_begin(), end = this->leaf_depth_end(); it != end; ++it) {
    LeafContainer& leafContainer = it.getLeafContainer();
    assert(!leafContainer.isBuilt());
    leafContainer.build(this->parameter_.nTrain, this->parameter_.k, this->parameter_.alpha,
                        this->parameter_.minimumVariance, alternateCentroids_);
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
  addFrame(frame);
  for (auto it = this->leaf_depth_begin(), end = this->leaf_depth_end(); it != end; ++it) {
    LeafContainer& leafContainer = it.getLeafContainer();
    if (!leafContainer.isBuilt()) {
      leafContainer.build(this->parameter_.nTrain, this->parameter_.k, this->parameter_.alpha,
                          this->parameter_.minimumVariance, alternateCentroids_);
    }
    bool isStatic = leafContainer.isStatic();

    //    if (isStatic && staticFrame) { staticFrame->push_back(leafContainer.getPoint()); }
    if (!isnan(leafContainer.getLastPoint().intensity)) {
      const float intensity = leafContainer.getLastPoint().intensity;

      if (dynamicFrame) {
        double probability = leafContainer.getProbability(intensity);

        bool isDynamic = probability < this->parameter_.dynamicThresholdLE;

        if (!isStatic && isDynamic) {
          PointT& point = leafContainer.getLastPoint();
          assert(!isnan(point.x));
          dynamicFrame->points.push_back(point);
        }
      }
    }
    leafContainer.updateModel(this->parameter_.alpha, this->parameter_.alpha * this->parameter_.nullAlphaRatio,
                              this->parameter_.minimumVariance);

    bool   updatedIsStatic = leafContainer.getStaticProbability() > this->parameter_.staticThresholdGT;
    PointT center;
    genLeafNodeCenterFromOctreeKey(it.getCurrentOctreeKey(), center);

    if (staticFrame && updatedIsStatic) { staticFrame->points.push_back(center); }
    if (staticFrameAdded && !isStatic && updatedIsStatic) { staticFrameAdded->points.push_back(center); }
    if (staticFrameRemoved && isStatic && !updatedIsStatic) { staticFrameRemoved->points.push_back(center); }

    leafContainer.setIsStatic(updatedIsStatic);
  }
}

}  // namespace jpcc::segmentation