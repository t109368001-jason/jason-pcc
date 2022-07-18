#include <jpcc/segmentation/JPCCSegmentationOPCGMMAdaptive.h>

#include <limits>

using namespace std;

namespace jpcc::segmentation {

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCSegmentationOPCGMMAdaptive::JPCCSegmentationOPCGMMAdaptive(const JPCCSegmentationParameter& parameter) :
    JPCCSegmentationBase(parameter), Base(parameter.resolution), isFirstFrame(true) {
  for (int i = -1; i >= -parameter_.k; i--) { alternateCentroids_.push_back(i / MAX_INTENSITY); }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCSegmentationOPCGMMAdaptive::appendTrainSamples(FramePtr frame) {
  for (auto it = leaf_depth_begin(), end = leaf_depth_end(); it != end; ++it) {
    it.getLeafContainer().resetLastPoint();
  }
  addFrame(frame);
  for (auto it = leaf_depth_begin(), end = leaf_depth_end(); it != end; ++it) {
    it.getLeafContainer().appendTrainSamples(parameter_.alpha);
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
                                                  FramePtr             staticFrame,
                                                  FramePtr             staticFrameAdded,
                                                  FramePtr             staticFrameRemoved) {
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
  if (staticFrameAdded) {
    staticFrameAdded->clear();
    staticFrameAdded->header.seq   = frame->header.seq;
    staticFrameAdded->header.stamp = frame->header.stamp;
  }
  if (staticFrameRemoved) {
    staticFrameRemoved->clear();
    staticFrameRemoved->header.seq   = frame->header.seq;
    staticFrameRemoved->header.stamp = frame->header.stamp;
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
    double          staticProbability = leafContainer.getGMM()->getStaticProbability();
    bool            isStatic          = staticProbability > parameter_.staticThresholdGT;
    PointXYZINormal staticPoint       = leafContainer.getAdaptivePoint();

    //    if (isStatic && staticFrame) { staticFrame->push_back(leafContainer.getPoint()); }
    if (!isnan(leafContainer.getIntensityNormalized())) {
      const float intensity = leafContainer.getIntensityNormalized();

      if (dynamicFrame) {
        double probability = leafContainer.getGMM()->getProbability(intensity);

        bool isDynamic = probability < parameter_.dynamicThresholdLE;

        if (!isStatic && isDynamic) {
          PointXYZINormal& point = leafContainer.getLastPoint();
          assert(!isnan(point.x));
          dynamicFrame->points.push_back(point);
        }
      }
    }
    leafContainer.updateModel(parameter_.alpha);

    bool             updatedIsStatic    = leafContainer.getGMM()->getStaticProbability() > parameter_.staticThresholdGT;
    PointXYZINormal& updatedStaticPoint = leafContainer.getAdaptivePoint();
    assert(!isnan(updatedStaticPoint.x));
    if (staticFrame && updatedIsStatic) { staticFrame->points.push_back(updatedStaticPoint); }
    if (staticFrameAdded && (!isStatic || isFirstFrame) && updatedIsStatic) {
      staticFrameAdded->points.push_back(updatedStaticPoint);
    }
    if (staticFrameRemoved && (isStatic && !isFirstFrame) && !updatedIsStatic) {
      assert(!isnan(staticPoint.x));
      staticFrameRemoved->points.push_back(staticPoint);
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
  if (staticFrameAdded) {
    staticFrameAdded->width  = staticFrameAdded->size();
    staticFrameAdded->height = 1;
  }
  if (staticFrameRemoved) {
    staticFrameRemoved->width  = staticFrameRemoved->size();
    staticFrameRemoved->height = 1;
  }
  isFirstFrame = false;
}

}  // namespace jpcc::segmentation