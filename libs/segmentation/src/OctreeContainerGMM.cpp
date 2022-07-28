#include "../include/jpcc/segmentation/OctreeContainerGMM.h"

using namespace std;
using namespace jpcc::math;

namespace jpcc::segmentation {

//////////////////////////////////////////////////////////////////////////////////////////////
OctreeContainerGMM::OctreeContainerGMM() : OctreeContainerLastPoint(), GMM() { OctreeContainerGMM::reset(); }

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerGMM::reset() {
  GMM::reset();
  trainSamples_ = make_shared<vector<float>>();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerGMM::addPoint(const PointXYZINormal& point) { OctreeContainerLastPoint::addPoint(point); }

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerGMM::addTrainSample() {
  if (isnan(lastPoint_.x)) { return; }
  if (!trainSamples_) { return; }
  trainSamples_->push_back(lastPoint_.intensity);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerGMM::build(const int                 nTrain,
                               const int                 K,
                               const double              alpha,
                               const double              minimumVariance,
                               const std::vector<float>& alternateCentroids) {
  for (auto& sample : *trainSamples_) {
    sample /= MAX_INTENSITY;
    assert(sample <= GMM_MAX_INTENSITY);
  }
  trainSamples_->resize(nTrain, GMM_NULL_INTENSITY);
  GMM::build(*trainSamples_, K, minimumVariance, {GMM_NULL_INTENSITY}, alternateCentroids);
  trainSamples_ = nullptr;
}

void OctreeContainerGMM::updateModel(const double alpha, const double nullAlpha, const double minimumVariance) {
  float intensity = getIntensityNormalized();
  if (isnan(intensity)) {
    GMM::updateModel(NULL_INTENSITY, nullAlpha, minimumVariance);
  } else {
    GMM::updateModel(intensity, alpha, minimumVariance);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
float OctreeContainerGMM::getIntensityNormalized() { return lastPoint_.intensity / MAX_INTENSITY; }

}  // namespace jpcc::segmentation