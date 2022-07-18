#include "../include/jpcc/segmentation/OctreeContainerGMM.h"

using namespace std;
using namespace jpcc::math;

namespace jpcc::segmentation {

//////////////////////////////////////////////////////////////////////////////////////////////
OctreeContainerGMM::OctreeContainerGMM() :
    OctreeContainerLastPoint(), trainSamples_(make_shared<vector<float>>()), gmm_() {}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerGMM::reset() {}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerGMM::addPoint(const PointXYZINormal& point) { OctreeContainerLastPoint::addPoint(point); }

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerGMM::addTrainSample() {
  if (isnan(lastPoint_.x)) { return; }
  if (!trainSamples_) { return; }
  trainSamples_->push_back(lastPoint_.intensity);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerGMM::build(const int           nTrain,
                               const int           K,
                               const double        alpha,
                               const double        minimumVariance,
                               std::vector<float>& alternateCentroids) {
  for (auto& sample : *trainSamples_) {
    sample /= MAX_INTENSITY;
    assert(sample <= GMM_MAX_INTENSITY);
  }
  trainSamples_->resize(nTrain, GMM_NULL_INTENSITY);
  gmm_          = jpcc::make_shared<GMM>(*trainSamples_, K, alpha, minimumVariance, alternateCentroids);
  trainSamples_ = nullptr;
}

void OctreeContainerGMM::updateModel() {
  gmm_->updateModel(isnan(getIntensityNormalized()) ? NULL_INTENSITY : getIntensityNormalized());
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool OctreeContainerGMM::isBuilt() const { return gmm_.operator bool(); }

//////////////////////////////////////////////////////////////////////////////////////////////
float OctreeContainerGMM::getIntensityNormalized() { return lastPoint_.intensity / MAX_INTENSITY; }

//////////////////////////////////////////////////////////////////////////////////////////////
const shared_ptr<std::vector<float>>& OctreeContainerGMM::getTrainSamples() const { return trainSamples_; }

//////////////////////////////////////////////////////////////////////////////////////////////
shared_ptr<std::vector<float>>& OctreeContainerGMM::getTrainSamples() { return trainSamples_; }

//////////////////////////////////////////////////////////////////////////////////////////////
const math::GMM::Ptr& OctreeContainerGMM::getGMM() const { return gmm_; }

//////////////////////////////////////////////////////////////////////////////////////////////
math::GMM::Ptr& OctreeContainerGMM::getGMM() { return gmm_; }

}  // namespace jpcc::octree