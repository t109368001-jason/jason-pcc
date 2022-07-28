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
void OctreeContainerGMM::addPoint(const PointXYZINormal& point) {
  assert(point.intensity <= MAX_INTENSITY);
  OctreeContainerLastPoint::addPoint(point);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerGMM::addTrainSample() {
  if (isnan(lastPoint_.x)) { return; }
  if (!trainSamples_) { return; }
  trainSamples_->push_back(lastPoint_.intensity);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerGMM::build(const int            nTrain,
                               const int            K,
                               const double         alpha,
                               const double         minimumVariance,
                               const vector<float>& alternateCentroids) {
  trainSamples_->resize(nTrain, NULL_INTENSITY);
  GMM::build(*trainSamples_, K, minimumVariance, {NULL_INTENSITY}, alternateCentroids);
  trainSamples_ = nullptr;
}

void OctreeContainerGMM::updateModel(const double alpha, const double nullAlpha, const double minimumVariance) {
  if (isnan(lastPoint_.intensity)) {
    GMM::updateModel(NULL_INTENSITY, nullAlpha, minimumVariance);
  } else {
    GMM::updateModel(lastPoint_.intensity, alpha, minimumVariance);
  }
}

}  // namespace jpcc::segmentation