#include <jpcc/segmentation/OctreeContainerGMM.h>

namespace jpcc::segmentation {

//////////////////////////////////////////////////////////////////////////////////////////////
OctreeContainerGMM::OctreeContainerGMM() : octree::OctreeContainerLastPoint<pcl::PointXYZI>(), GMM() {
  OctreeContainerGMM::reset();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerGMM::reset() {
  GMM::reset();
  trainSamples_ = make_shared<std::vector<Intensity>>();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerGMM::addPoint(const pcl::PointXYZI& point) {
  assert(point.intensity <= MAX_INTENSITY);
  octree::OctreeContainerLastPoint<pcl::PointXYZI>::addPoint(point);
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool OctreeContainerGMM::isBuilt(const int index) const { return GMM::isBuilt(); }

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerGMM::addTrainSample() {
  if (std::isnan(this->lastPoint_.x)) { return; }
  if (!trainSamples_) { return; }
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
bool OctreeContainerGMM::isStatic(const std::vector<double>& staticThresholdVector,
                                  const std::vector<double>& nullStaticThresholdVector,
                                  const std::vector<bool>&   outputExistsPointOnlyVector) {
  if (isnan(this->lastPoint_.intensity)) {
    if (!outputExistsPointOnlyVector.front()) {
      if (GMM::getStaticProbability() > nullStaticThresholdVector.front()) { return true; }
    }
    return false;
  } else {
    if (GMM::getProbability(Intensity(this->lastPoint_.intensity)) > staticThresholdVector.front()) { return true; }
    return false;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerGMM::updateModel(const int    index,
                                     const double alpha,
                                     const double nullAlpha,
                                     const double minimumVariance) {
  if (std::isnan(this->lastPoint_.intensity)) {
    GMM::updateModel(NULL_INTENSITY, nullAlpha, minimumVariance);
  } else {
    GMM::updateModel(Intensity(this->lastPoint_.intensity), alpha, minimumVariance);
  }
}

}  // namespace jpcc::segmentation