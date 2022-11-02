using namespace std;

namespace jpcc::octree {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
OctreeContainerLastPoint<PointT>::OctreeContainerLastPoint() : OctreeContainerBase() {
  OctreeContainerLastPoint::reset();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void OctreeContainerLastPoint<PointT>::reset() {
  lastPoint_.x = numeric_limits<float>::quiet_NaN();
  lastPoint_.y = numeric_limits<float>::quiet_NaN();
  lastPoint_.z = numeric_limits<float>::quiet_NaN();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void OctreeContainerLastPoint<PointT>::addPoint(const PointT& point) {
  lastPoint_ = point;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
const PointT& OctreeContainerLastPoint<PointT>::getLastPoint() const {
  return lastPoint_;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
PointT& OctreeContainerLastPoint<PointT>::getLastPoint() {
  return lastPoint_;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void OctreeContainerLastPoint<PointT>::resetLastPoint() {
  OctreeContainerLastPoint::reset();
}

}  // namespace jpcc::octree