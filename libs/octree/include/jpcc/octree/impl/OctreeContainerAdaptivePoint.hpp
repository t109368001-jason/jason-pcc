using namespace std;

namespace jpcc::octree {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
OctreeContainerAdaptivePoint<PointT>::OctreeContainerAdaptivePoint() : OctreeContainerLastPoint<PointT>() {
  OctreeContainerAdaptivePoint::reset();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void OctreeContainerAdaptivePoint<PointT>::reset() {
  adaptivePoint_.x = numeric_limits<float>::quiet_NaN();
  adaptivePoint_.y = numeric_limits<float>::quiet_NaN();
  adaptivePoint_.z = numeric_limits<float>::quiet_NaN();
  if constexpr (pcl::traits::has_intensity_v<PointT>) { adaptivePoint_.intensity = numeric_limits<float>::quiet_NaN(); }
  adaptivePoint_.normal_x  = numeric_limits<float>::quiet_NaN();
  adaptivePoint_.normal_y  = numeric_limits<float>::quiet_NaN();
  adaptivePoint_.normal_z  = numeric_limits<float>::quiet_NaN();
  adaptivePoint_.curvature = numeric_limits<float>::quiet_NaN();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void OctreeContainerAdaptivePoint<PointT>::addPoint(const PointT& point) {
  OctreeContainerLastPoint<PointT>::addPoint(point);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void OctreeContainerAdaptivePoint<PointT>::updateAdaptivePoint(double alpha) {
  if (isnan(this->lastPoint_.x)) { return; }
  if (isnan(adaptivePoint_.x)) {
    adaptivePoint_.x = this->lastPoint_.x;
    adaptivePoint_.y = this->lastPoint_.y;
    adaptivePoint_.z = this->lastPoint_.z;
    if constexpr (pcl::traits::has_intensity_v<PointT>) { adaptivePoint_.intensity = this->lastPoint_.intensity; }
    if constexpr (pcl::traits::has_normal_v<PointT>) {
      adaptivePoint_.normal_x  = this->lastPoint_.normal_x;
      adaptivePoint_.normal_y  = this->lastPoint_.normal_y;
      adaptivePoint_.normal_z  = this->lastPoint_.normal_z;
      adaptivePoint_.curvature = this->lastPoint_.curvature;
    }
  } else {
    adaptivePoint_.x = (1.0 - alpha) * adaptivePoint_.x + alpha * this->lastPoint_.x;
    adaptivePoint_.y = (1.0 - alpha) * adaptivePoint_.y + alpha * this->lastPoint_.y;
    adaptivePoint_.z = (1.0 - alpha) * adaptivePoint_.z + alpha * this->lastPoint_.z;
    if constexpr (pcl::traits::has_intensity_v<PointT>) {
      adaptivePoint_.intensity = (1.0 - alpha) * adaptivePoint_.intensity + alpha * this->lastPoint_.intensity;
    }
    if constexpr (pcl::traits::has_normal_v<PointT>) {
      adaptivePoint_.normal_x = (1.0 - alpha) * adaptivePoint_.normal_x + alpha * this->lastPoint_.normal_x;
      adaptivePoint_.normal_y = (1.0 - alpha) * adaptivePoint_.normal_y + alpha * this->lastPoint_.normal_y;
      adaptivePoint_.normal_z = (1.0 - alpha) * adaptivePoint_.normal_z + alpha * this->lastPoint_.normal_z;
    }
  }
  this->resetLastPoint();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
const PointT& OctreeContainerAdaptivePoint<PointT>::getAdaptivePoint() const {
  return adaptivePoint_;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
PointT& OctreeContainerAdaptivePoint<PointT>::getAdaptivePoint() {
  return adaptivePoint_;
}

}  // namespace jpcc::octree