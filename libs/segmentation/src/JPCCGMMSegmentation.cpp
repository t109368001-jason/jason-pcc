#include <jpcc/segmentation/JPCCGMMSegmentation.h>

#include <execution>
#include <utility>

using namespace std;
using namespace jpcc::octree;

namespace jpcc::segmentation {

JPCCGMMSegmentation::JPCCGMMSegmentation(JPCCGMMSegmentationParameter parameter) :
    parameter_(std::move(parameter)), octree_(parameter_.resolution) {
  for (int i = -1; i >= -parameter_.k; i--) { alternateCentroids_.push_back(i / MAX_INTENSITY); }
}

//////////////////////////////////////////////////////////////////////////////////////////////
size_t JPCCGMMSegmentation::getNTrain() const { return parameter_.nTrain; }

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCGMMSegmentation::appendTrainSamples(const GroupOfFrame& groupOfFrame) {
  for (const auto& frame : groupOfFrame) {
    octree_.addFrame(frame);
    for (auto it = octree_.leaf_depth_begin(), end = octree_.leaf_depth_end(); it != end; ++it) {
      it.getLeafContainer().setIntensity(numeric_limits<float>::quiet_NaN());
    }
  }
  for (auto it = octree_.leaf_depth_begin(), end = octree_.leaf_depth_end(); it != end; ++it) {
    it.getLeafContainer().updatePoint(parameter_.alpha);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCGMMSegmentation::build() {
  for (auto it = octree_.leaf_depth_begin(), end = octree_.leaf_depth_end(); it != end; ++it) {
    LeafContainerT& leafContainer = it.getLeafContainer();
    for (auto& sample : *leafContainer.getTrainSamples()) {
      sample /= MAX_INTENSITY;
      assert(sample <= GMM_MAX_INTENSITY);
    }
    leafContainer.getTrainSamples()->resize(parameter_.nTrain, GMM_NULL_INTENSITY);
    leafContainer.initGMM(parameter_.k, parameter_.alpha, parameter_.minimumVariance, alternateCentroids_);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCGMMSegmentation::segmentation(const FrameConstPtr& frame, FramePtr dynamicFrame, FramePtr staticFrame) {
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

  for (auto it = octree_.leaf_depth_begin(), end = octree_.leaf_depth_end(); it != end; ++it) {
    it.getLeafContainer().setIntensity(std::numeric_limits<float>::quiet_NaN());
  }
  octree_.addFrame(frame);
  for (auto it = octree_.leaf_depth_begin(), end = octree_.leaf_depth_end(); it != end; ++it) {
    LeafContainerT& leafContainer = it.getLeafContainer();
    if (!leafContainer.getGMM()) {
      leafContainer.getTrainSamples()->resize(parameter_.nTrain, GMM_NULL_INTENSITY);
      leafContainer.initGMM(parameter_.k, parameter_.alpha, parameter_.minimumVariance, alternateCentroids_);
    }

    leafContainer.updatePoint(parameter_.alpha);

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
          PointXYZINormal& point = leafContainer.getPoint();
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
        PointXYZINormal& point = leafContainer.getPoint();
        assert(!isnan(point.x));
        staticFrame->points.push_back(point);
      }
    }

    // reset
    leafContainer.setIntensity(numeric_limits<float>::quiet_NaN());
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