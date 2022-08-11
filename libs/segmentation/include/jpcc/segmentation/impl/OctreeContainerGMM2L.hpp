namespace jpcc::segmentation {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
OctreeContainerGMM2L<PointT>::OctreeContainerGMM2L() : octree::OctreeContainerLastPoint<PointT>() {
  static_assert(pcl::traits::has_intensity_v<PointT>, "invalid template type");
  OctreeContainerGMM2L::reset();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void OctreeContainerGMM2L<PointT>::reset() {
  for (auto& gmm : gmms_) { gmm.reset(); }
  for (auto& trainSamples : trainSamplesVector_) { trainSamples = make_shared<std::vector<float>>(); }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void OctreeContainerGMM2L<PointT>::addPoint(const PointT& point) {
  assert(point.intensity <= MAX_INTENSITY);
  octree::OctreeContainerLastPoint<PointT>::addPoint(point);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool OctreeContainerGMM2L<PointT>::isBuilt(const int index) const {
  return gmms_.at(index).isBuilt();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void OctreeContainerGMM2L<PointT>::addTrainSample() {
  if (std::isnan(this->lastPoint_.x)) { return; }
  for (auto& trainSamples : trainSamplesVector_) {
    if (!trainSamples) { continue; }
    trainSamples->push_back(this->lastPoint_.intensity);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void OctreeContainerGMM2L<PointT>::build(const int                 index,
                                         const int                 nTrain,
                                         const int                 K,
                                         const double              alpha,
                                         const double              minimumVariance,
                                         const std::vector<float>& alternateCentroids) {
  trainSamplesVector_.at(index)->resize(nTrain, NULL_INTENSITY);
  gmms_.at(index).build(*trainSamplesVector_.at(index), K, minimumVariance, {NULL_INTENSITY}, alternateCentroids);
  trainSamplesVector_.at(index) = nullptr;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool OctreeContainerGMM2L<PointT>::isStatic(std::vector<double> dynamicThresholdLEVector,
                                            std::vector<double> staticThresholdGTVector) {
  if (isnan(this->lastPoint_.intensity)) {
    return gmms_.at(LONG_INDEX).getStaticProbability() > staticThresholdGTVector.at(LONG_INDEX);
  } else {
    if (gmms_.at(SHORT_INDEX).getProbability(this->lastPoint_.intensity) > dynamicThresholdLEVector.at(SHORT_INDEX)) {
      return true;
    }
    if (gmms_.at(LONG_INDEX).getProbability(this->lastPoint_.intensity) > dynamicThresholdLEVector.at(LONG_INDEX)) {
      return true;
    }
    return false;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void OctreeContainerGMM2L<PointT>::updateModel(const int    index,
                                               const double alpha,
                                               const double nullAlpha,
                                               const double minimumVariance) {
  if (std::isnan(this->lastPoint_.intensity)) {
    gmms_.at(index).updateModel(NULL_INTENSITY, nullAlpha, minimumVariance);
  } else {
    gmms_.at(index).updateModel(this->lastPoint_.intensity, alpha, minimumVariance);
  }
}

}  // namespace jpcc::segmentation