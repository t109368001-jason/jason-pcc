using namespace std;
using namespace jpcc::math;

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
  trainSamples_ = make_shared<vector<float>>();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void OctreeContainerGMM<PointT>::addPoint(const PointT& point) {
  assert(point.intensity <= MAX_INTENSITY);
  octree::OctreeContainerLastPoint<PointT>::addPoint(point);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void OctreeContainerGMM<PointT>::addTrainSample() {
  if (isnan(this->lastPoint_.x)) { return; }
  if (!trainSamples_) { return; }
  trainSamples_->push_back(this->lastPoint_.intensity);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void OctreeContainerGMM<PointT>::build(const int            nTrain,
                                       const int            K,
                                       const double         alpha,
                                       const double         minimumVariance,
                                       const vector<float>& alternateCentroids) {
  trainSamples_->resize(nTrain, NULL_INTENSITY);
  GMM::build(*trainSamples_, K, minimumVariance, {NULL_INTENSITY}, alternateCentroids);
  trainSamples_ = nullptr;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void OctreeContainerGMM<PointT>::updateModel(const double alpha, const double nullAlpha, const double minimumVariance) {
  if (isnan(this->lastPoint_.intensity)) {
    GMM::updateModel(NULL_INTENSITY, nullAlpha, minimumVariance);
  } else {
    GMM::updateModel(this->lastPoint_.intensity, alpha, minimumVariance);
  }
}

}  // namespace jpcc::segmentation