namespace jpcc::octree {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
OctreeContainerPointNormals<PointT>::OctreeContainerPointNormals() : OctreeContainerBase() {
  OctreeContainerPointNormals::reset();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void OctreeContainerPointNormals<PointT>::reset() {
  azimuths_.clear();
  zeniths_.clear();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void OctreeContainerPointNormals<PointT>::addPoint(const PointT& point) {
  if constexpr (!pcl::traits::has_normal_v<PointT>) {
    static_assert(dependent_false_v<PointT>, "invalid template type");
  }
  if (std::isnan(point.normal_x)) {
    return;
  }
  double azimuth;
  if (point.normal_x > 0) {
    azimuth = std::atan2(point.normal_y, point.normal_x);
  } else if (point.normal_x < 0 && point.normal_y >= 0) {
    azimuth = std::atan2(point.normal_y, point.normal_x) + M_PI;
  } else if (point.normal_x < 0 && point.normal_y < 0) {
    azimuth = std::atan2(point.normal_y, point.normal_x) - M_PI;
  } else if (point.normal_x == 0 && point.normal_y > 0) {
    azimuth = M_PI_2;
  } else if (point.normal_x == 0 && point.normal_y < 0) {
    azimuth = -M_PI_2;
  } else {
    azimuth = -0;
  }
  double zenith = std::acos(point.normal_z);

  assert(!std::isnan(azimuth));
  assert(!std::isnan(zenith));

  if (azimuth < 0) {
    azimuth += M_PI + M_PI;
  }

  azimuths_.push_back(azimuth);
  zeniths_.push_back(zenith);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
const std::vector<double>& OctreeContainerPointNormals<PointT>::getAzimuths() const {
  return azimuths_;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
const std::vector<double>& OctreeContainerPointNormals<PointT>::getZeniths() const {
  return zeniths_;
}

}  // namespace jpcc::octree