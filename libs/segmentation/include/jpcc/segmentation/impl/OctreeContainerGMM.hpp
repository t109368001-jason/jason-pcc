namespace jpcc::segmentation {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
OctreeContainerGMM<PointT>::OctreeContainerGMM() : octree::OctreeContainerLastPoint<PointT>(), GMM() {
  static_assert(pcl::traits::has_intensity_v<PointT>, "invalid template type");
  OctreeContainerGMM::reset();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void OctreeContainerGMM<PointT>::reset() {
  GMM::reset();
  trainSamples_ = make_shared<std::vector<float>>();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void OctreeContainerGMM<PointT>::addPoint(const PointT& point) {
  assert(point.intensity <= MAX_INTENSITY);
  octree::OctreeContainerLastPoint<PointT>::addPoint(point);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool OctreeContainerGMM<PointT>::isBuilt(const int index) const {
  return GMM::isBuilt();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void OctreeContainerGMM<PointT>::addTrainSample() {
  if (std::isnan(this->lastPoint_.x)) { return; }
  if (!trainSamples_) { return; }
  trainSamples_->push_back(this->lastPoint_.intensity);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void OctreeContainerGMM<PointT>::build(const int                 index,
                                       const int                 nTrain,
                                       const int                 K,
                                       const double              alpha,
                                       const double              minimumVariance,
                                       const std::vector<float>& alternateCentroids) {
  GMM::buildN(*trainSamples_, K, nTrain, minimumVariance, NULL_INTENSITY, alternateCentroids);
  trainSamples_ = nullptr;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool OctreeContainerGMM<PointT>::isStatic(const std::vector<double>& staticThresholdVector,
                                          const std::vector<double>& nullStaticThresholdVector,
                                          const std::vector<bool>&   outputExistsPointOnlyVector) {
  if (isnan(this->lastPoint_.intensity)) {
    if (!outputExistsPointOnlyVector.at(0)) {
      if (GMM::getStaticProbability() > nullStaticThresholdVector.at(0)) { return true; }
    }
    return false;
  } else {
    if (GMM::getProbability(this->lastPoint_.intensity) > staticThresholdVector.at(0)) { return true; }
    return false;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void OctreeContainerGMM<PointT>::updateModel(const int    index,
                                             const double alpha,
                                             const double nullAlpha,
                                             const double minimumVariance) {
  if (std::isnan(this->lastPoint_.intensity)) {
    GMM::updateModel(NULL_INTENSITY, nullAlpha, minimumVariance);
  } else {
    GMM::updateModel(this->lastPoint_.intensity, alpha, minimumVariance);
  }
}

}  // namespace jpcc::segmentation