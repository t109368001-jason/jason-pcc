#include <jpcc/segmentation/OctreeContainerGMM.h>

namespace jpcc::segmentation {

//////////////////////////////////////////////////////////////////////////////////////////////
OctreeContainerGMM::OctreeContainerGMM() : octree::OctreeContainerLastPoint<PointSegmentation>(), GMM() {
  OctreeContainerGMM::reset();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerGMM::reset() {
  GMM::reset();
  trainSamples_ = make_shared<std::vector<Intensity>>();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerGMM::addPoint(const PointSegmentation& point) {
  assert(point.intensity <= MAX_INTENSITY);
  octree::OctreeContainerLastPoint<PointSegmentation>::addPoint(point);
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool OctreeContainerGMM::isBuilt(const int index) const {
  return GMM::isBuilt();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerGMM::addTrainSample() {
  if (std::isnan(this->lastPoint_.x)) {
    return;
  }
  if (!trainSamples_) {
    return;
  }
  trainSamples_->push_back(Intensity(this->lastPoint_.intensity));
}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerGMM::build(const int                  index,
                               const int                  nTrain,
                               const int                  K,
                               const double               alpha,
                               const double               minimumVariance,
                               const std::set<Intensity>& alternateCentroids) {
  GMM::buildN(*trainSamples_, K, nTrain, minimumVariance, NULL_INTENSITY, alternateCentroids);
  trainSamples_ = nullptr;
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool OctreeContainerGMM::isStatic(const std::vector<double>& staticThreshold1Vector,
                                  const std::vector<double>& staticThreshold2Vector,
                                  const std::vector<double>& nullStaticThreshold1Vector,
                                  const std::vector<double>& nullStaticThreshold2Vector,
                                  const bool                 lastIsStatic) {
  Intensity intensity = isnan(this->lastPoint_.x) ? NULL_INTENSITY : static_cast<Intensity>(this->lastPoint_.intensity);
  if (!lastIsStatic) {
    if (GMM::getProbability(intensity) > staticThreshold1Vector.front()) {
      return true;
    }
  } else {
    if (GMM::getProbability(intensity) > staticThreshold2Vector.front()) {
      return true;
    }
  }
  return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerGMM::updateModel(int index, double alpha, double minimumVariance) {
  Intensity intensity = isnan(this->lastPoint_.x) ? NULL_INTENSITY : static_cast<Intensity>(this->lastPoint_.intensity);
  GMM::updateModel(intensity, alpha, minimumVariance);
}

}  // namespace jpcc::segmentation