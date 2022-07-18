#include <jpcc/segmentation/JPCCSegmentationOPCGMMAdaptive.h>

#include <limits>

using namespace std;

namespace jpcc::segmentation {

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCSegmentationOPCGMMAdaptive::JPCCSegmentationOPCGMMAdaptive(const JPCCSegmentationParameter& parameter) :
    JPCCSegmentationBase(parameter), Base(parameter.resolution) {
  for (int i = -1; i >= -parameter_.k; i--) { alternateCentroids_.push_back(i / MAX_INTENSITY); }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCSegmentationOPCGMMAdaptive::appendTrainSamples(FramePtr frame) {
  for (auto it = leaf_depth_begin(), end = leaf_depth_end(); it != end; ++it) {
    it.getLeafContainer().resetLastPoint();
  }
  addFrame(frame);
  for (auto it = leaf_depth_begin(), end = leaf_depth_end(); it != end; ++it) {
    it.getLeafContainer().update(parameter_.alpha);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCSegmentationOPCGMMAdaptive::build() {
  for (auto it = leaf_depth_begin(), end = leaf_depth_end(); it != end; ++it) {
    LeafContainer& leafContainer = it.getLeafContainer();
    assert(!leafContainer.isBuilt());
    leafContainer.build(parameter_.nTrain, parameter_.k, parameter_.alpha, parameter_.minimumVariance,
                        alternateCentroids_);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCSegmentationOPCGMMAdaptive::segmentation(const FrameConstPtr& frame,
                                                  FramePtr             dynamicFrame,
                                                  FramePtr             staticFrame) {
  if (dynamicFrame) {
    dynamicFrame->clear();
    dynamicFrame->header.seq   = frame->header.seq;
    dynamicFrame->header.stamp = frame->header.stamp;
  }
  if (staticFrame) {
    staticFrame->clear();
    staticFrame->header.seq   = frame->header.seq;
    staticFrame->header.stamp = frame->header.stamp;
  }

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

    leafContainer.update(parameter_.alpha);

    //    if (isStatic && staticFrame) { staticFrame->push_back(leafContainer.getPoint()); }
    if (!isnan(leafContainer.getIntensity())) {
      const float intensity = leafContainer.getIntensity() / MAX_INTENSITY;
      assert(intensity <= GMM_MAX_INTENSITY);

      if (dynamicFrame) {
        double probability       = leafContainer.getGMM()->getProbability(intensity);
        double staticProbability = leafContainer.getGMM()->getStaticProbability();

        bool isDynamic = probability < parameter_.dynamicThresholdLE;
        bool isStatic  = staticProbability > parameter_.staticThresholdGT;

        if (!isStatic && isDynamic) {
          PointXYZINormal& point = leafContainer.getLastPoint();
          assert(!isnan(point.x));
          dynamicFrame->points.push_back(point);
        }
      }

      // update model
      leafContainer.getGMM()->updateModel(intensity);
    } else {
      // update model
      leafContainer.getGMM()->updateModel(NULL_INTENSITY);
    }

    if (staticFrame) {
      bool updatedIsStatic = leafContainer.getGMM()->getStaticProbability() > parameter_.staticThresholdGT;
      if (updatedIsStatic) {
        PointXYZINormal& point = leafContainer.getAdaptivePoint();
        assert(!isnan(point.x));
        staticFrame->points.push_back(point);
      }
    }
  }

  if (dynamicFrame) {
    dynamicFrame->width  = dynamicFrame->size();
    dynamicFrame->height = 1;
  }
  if (staticFrame) {
    staticFrame->width  = staticFrame->size();
    staticFrame->height = 1;
  }
}

}  // namespace jpcc::segmentation