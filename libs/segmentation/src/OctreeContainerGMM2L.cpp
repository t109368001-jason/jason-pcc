#include <jpcc/segmentation/OctreeContainerGMM2L.h>

namespace jpcc::segmentation {

//////////////////////////////////////////////////////////////////////////////////////////////
OctreeContainerGMM2L::OctreeContainerGMM2L() : octree::OctreeContainerLastPoint<pcl::PointXYZI>() {
  OctreeContainerGMM2L::reset();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerGMM2L::reset() {
  for (auto& gmm : gmmArray_) { gmm.reset(); }
  for (auto& trainSamples : trainSamplesArray_) { trainSamples = make_shared<std::vector<Intensity>>(); }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerGMM2L::addPoint(const pcl::PointXYZI& point) {
  assert(point.intensity <= MAX_INTENSITY);
  octree::OctreeContainerLastPoint<pcl::PointXYZI>::addPoint(point);
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool OctreeContainerGMM2L::isBuilt(const int index) const { return gmmArray_[index].isBuilt(); }

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerGMM2L::addTrainSample() {
  if (std::isnan(this->lastPoint_.x)) { return; }
  for (auto& trainSamples : trainSamplesArray_) {
    if (!trainSamples) { continue; }
    trainSamples->push_back(Intensity(this->lastPoint_.intensity));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerGMM2L::build(const int                  index,
                                 const int                  nTrain,
                                 const int                  K,
                                 const double               alpha,
                                 const double               minimumVariance,
                                 const std::set<Intensity>& alternateCentroids) {
  gmmArray_[index].buildN(*trainSamplesArray_[index], K, nTrain, minimumVariance, NULL_INTENSITY, alternateCentroids);
  trainSamplesArray_[index] = nullptr;
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool OctreeContainerGMM2L::isStatic(const std::vector<double>& staticThresholdVector,
                                    const std::vector<double>& nullStaticThresholdVector,
                                    const std::vector<bool>&   outputExistsPointOnlyVector) {
  if (isnan(this->lastPoint_.intensity)) {
    if (!outputExistsPointOnlyVector[LONG_INDEX]) {
      if (gmmArray_[LONG_INDEX].getStaticProbability() > nullStaticThresholdVector[LONG_INDEX]) { return true; }
    }
    if (!outputExistsPointOnlyVector[SHORT_INDEX]) {
      if (gmmArray_[SHORT_INDEX].getStaticProbability() > nullStaticThresholdVector[SHORT_INDEX]) { return true; }
    }
    return false;
  } else {
    if (gmmArray_[SHORT_INDEX].getProbability(Intensity(this->lastPoint_.intensity)) >
        staticThresholdVector[SHORT_INDEX]) {
      return true;
    }
    if (gmmArray_[LONG_INDEX].getProbability(Intensity(this->lastPoint_.intensity)) >
        staticThresholdVector[LONG_INDEX]) {
      return true;
    }
    return false;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerGMM2L::updateModel(const int    index,
                                       const double alpha,
                                       const double nullAlpha,
                                       const double minimumVariance) {
  if (std::isnan(this->lastPoint_.intensity)) {
    gmmArray_[index].updateModel(NULL_INTENSITY, nullAlpha, minimumVariance);
  } else {
    gmmArray_[index].updateModel(Intensity(this->lastPoint_.intensity), alpha, minimumVariance);
  }
}

}  // namespace jpcc::segmentation