#include <jpcc/segmentation/JPCCGMMSegmentation.h>

#include <execution>
#include <utility>

using namespace std;
using namespace jpcc::octree;

namespace jpcc::segmentation {

JPCCGMMSegmentation::JPCCGMMSegmentation(JPCCGMMSegmentationParameter parameter) :
    parameter_(std::move(parameter)), octree_(parameter_.resolution) {
  for (int i = -1; i >= -parameter_.k; i--) { alternateCentroids_.push_back(i); }
}

//////////////////////////////////////////////////////////////////////////////////////////////
size_t JPCCGMMSegmentation::getNTrain() const { return parameter_.nTrain; }

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCGMMSegmentation::appendTrainSamples(const GroupOfFrame& groupOfFrame) {
  for_each(groupOfFrame.begin(), groupOfFrame.end(), [this](const auto& frame) { octree_.addFrame(frame); });
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCGMMSegmentation::build() {
  for (auto it = octree_.leaf_depth_begin(), end = octree_.leaf_depth_end(); it != end; ++it) {
    LeafContainerT& leafContainer = it.getLeafContainer();
    leafContainer.getTrainSamples()->resize(parameter_.nTrain, NULL_INTENSITY);
    leafContainer.initGMM(parameter_.k, parameter_.alpha, parameter_.minimumVariance, alternateCentroids_);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCGMMSegmentation::segmentation(const FrameConstPtr& frame, FramePtr dynamicFrame, FramePtr staticFrame) {
  if (dynamicFrame) { dynamicFrame->clear(); }
  if (staticFrame) { staticFrame->clear(); }

  for (auto it = octree_.leaf_depth_begin(), end = octree_.leaf_depth_end(); it != end; ++it) {
    LeafContainerT& leafContainer = it.getLeafContainer();
    leafContainer.setIntensity(std::numeric_limits<float>::quiet_NaN());
  }
  octree_.addFrame(frame);
  for (auto it = octree_.leaf_depth_begin(), end = octree_.leaf_depth_end(); it != end; ++it) {
    LeafContainerT& leafContainer = it.getLeafContainer();
    const float     intensity     = leafContainer.getIntensity();

    leafContainer.updatePoint(parameter_.alpha);

    double staticProbability = leafContainer.getGMM()->getStaticProbability();

    bool isStatic = staticProbability > parameter_.staticThresholdGT;

    //    if (isStatic && staticFrame) { staticFrame->push_back(leafContainer.getPoint()); }
    if (!isnan(intensity)) {
      double probability = leafContainer.getGMM()->getProbability(intensity);
      bool   isDynamic   = probability < parameter_.dynamicThresholdLE;
      if (!isStatic && isDynamic && dynamicFrame) { dynamicFrame->push_back(leafContainer.getPoint()); }
      // update model
      leafContainer.getGMM()->updateModel(intensity);
    } else {
      // update model
      leafContainer.getGMM()->updateModel(NULL_INTENSITY);
    }

    bool updatedIsStatic = leafContainer.getGMM()->getStaticProbability() > parameter_.staticThresholdGT;
    if (updatedIsStatic && staticFrame) { staticFrame->push_back(leafContainer.getPoint()); }

    // reset
    leafContainer.setIntensity(numeric_limits<float>::quiet_NaN());
  }
}

}  // namespace jpcc::segmentation