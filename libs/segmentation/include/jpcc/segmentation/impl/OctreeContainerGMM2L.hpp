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
  for (auto& gmm : gmmArray_) { gmm.reset(); }
  for (auto& trainSamples : trainSamplesArray_) { trainSamples = make_shared<std::vector<float>>(); }
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
  return gmmArray_[index].isBuilt();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void OctreeContainerGMM2L<PointT>::addTrainSample() {
  if (std::isnan(this->lastPoint_.x)) { return; }
  for (auto& trainSamples : trainSamplesArray_) {
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
  gmmArray_[index].buildN(*trainSamplesArray_[index], K, nTrain, minimumVariance, NULL_INTENSITY, alternateCentroids);
  trainSamplesArray_[index] = nullptr;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool OctreeContainerGMM2L<PointT>::isStatic(const std::vector<double>& staticThresholdVector,
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
    if (gmmArray_[SHORT_INDEX].getProbability(this->lastPoint_.intensity) > staticThresholdVector[SHORT_INDEX]) {
      return true;
    }
    if (gmmArray_[LONG_INDEX].getProbability(this->lastPoint_.intensity) > staticThresholdVector[LONG_INDEX]) {
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
    gmmArray_[index].updateModel(NULL_INTENSITY, nullAlpha, minimumVariance);
  } else {
    gmmArray_[index].updateModel(this->lastPoint_.intensity, alpha, minimumVariance);
  }
}

}  // namespace jpcc::segmentation