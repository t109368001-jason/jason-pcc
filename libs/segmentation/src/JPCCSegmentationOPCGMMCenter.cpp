#include <jpcc/segmentation/JPCCSegmentationOPCGMMCemter.h>

#include <limits>

using namespace std;

namespace jpcc::segmentation {

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCSegmentationOPCGMMCemter::JPCCSegmentationOPCGMMCemter(const JPCCSegmentationParameter& parameter) :
    JPCCSegmentationBase(parameter), Base(parameter.resolution) {
  for (int i = -1; i >= -parameter_.k; i--) { alternateCentroids_.push_back(static_cast<float>(i) / MAX_INTENSITY); }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCSegmentationOPCGMMCemter::appendTrainSamples(FramePtr frame) {
  for (auto it = leaf_depth_begin(), end = leaf_depth_end(); it != end; ++it) {
    it.getLeafContainer().resetLastPoint();
  }
  addFrame(frame);
  for (auto it = leaf_depth_begin(), end = leaf_depth_end(); it != end; ++it) {
    it.getLeafContainer().addTrainSample();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCSegmentationOPCGMMCemter::build() {
  for (auto it = leaf_depth_begin(), end = leaf_depth_end(); it != end; ++it) {
    LeafContainer& leafContainer = it.getLeafContainer();
    assert(!leafContainer.isBuilt());
    leafContainer.build(parameter_.nTrain, parameter_.k, parameter_.alpha, parameter_.minimumVariance,
                        alternateCentroids_);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCSegmentationOPCGMMCemter::segmentation(const FrameConstPtr& frame,
                                                FramePtr             dynamicFrame,
                                                FramePtr             staticFrame,
                                                FramePtr             staticFrameAdded,
                                                FramePtr             staticFrameRemoved) {
  for (auto it = leaf_depth_begin(), end = leaf_depth_end(); it != end; ++it) {
    it.getLeafContainer().resetLastPoint();
  }
  addFrame(frame);
  for (auto it = leaf_depth_begin(), end = leaf_depth_end(); it != end; ++it) {
    LeafContainer& leafContainer = it.getLeafContainer();
    if (!leafContainer.isBuilt()) {
      leafContainer.build(parameter_.nTrain, parameter_.k, parameter_.alpha, parameter_.minimumVariance,
                          alternateCentroids_);
    }
    bool isStatic = leafContainer.isStatic();

    //    if (isStatic && staticFrame) { staticFrame->push_back(leafContainer.getPoint()); }
    if (!isnan(leafContainer.getLastPoint().intensity)) {
      const float intensity = leafContainer.getLastPoint().intensity;

      if (dynamicFrame) {
        double probability = leafContainer.getProbability(intensity);

        bool isDynamic = probability < parameter_.dynamicThresholdLE;

        if (!isStatic && isDynamic) {
          PointXYZINormal& point = leafContainer.getLastPoint();
          assert(!isnan(point.x));
          dynamicFrame->points.push_back(point);
        }
      }
    }
    leafContainer.updateModel(parameter_.alpha, parameter_.alpha * parameter_.nullAlphaRatio,
                              parameter_.minimumVariance);

    bool            updatedIsStatic = leafContainer.getStaticProbability() > parameter_.staticThresholdGT;
    PointXYZINormal center;
    genLeafNodeCenterFromOctreeKey(it.getCurrentOctreeKey(), center);

    if (staticFrame && updatedIsStatic) { staticFrame->points.push_back(center); }
    if (staticFrameAdded && !isStatic && updatedIsStatic) { staticFrameAdded->points.push_back(center); }
    if (staticFrameRemoved && isStatic && !updatedIsStatic) { staticFrameRemoved->points.push_back(center); }

    leafContainer.setIsStatic(updatedIsStatic);
  }
}

}  // namespace jpcc::segmentation