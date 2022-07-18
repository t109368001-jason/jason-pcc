#include <jpcc/octree/OctreeContainerLastPoint.h>

using namespace std;

namespace jpcc::octree {

//////////////////////////////////////////////////////////////////////////////////////////////
OctreeContainerLastPoint::OctreeContainerLastPoint() : lastPoint_() { reset(); }

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerLastPoint::reset() {
  lastPoint_.x         = numeric_limits<float>::quiet_NaN();
  lastPoint_.y         = numeric_limits<float>::quiet_NaN();
  lastPoint_.z         = numeric_limits<float>::quiet_NaN();
  lastPoint_.intensity = numeric_limits<float>::quiet_NaN();
  lastPoint_.normal_x  = numeric_limits<float>::quiet_NaN();
  lastPoint_.normal_y  = numeric_limits<float>::quiet_NaN();
  lastPoint_.normal_z  = numeric_limits<float>::quiet_NaN();
  lastPoint_.curvature = numeric_limits<float>::quiet_NaN();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerLastPoint::addPoint(const PointXYZINormal& point) { lastPoint_ = point; }

//////////////////////////////////////////////////////////////////////////////////////////////
const PointXYZINormal& OctreeContainerLastPoint::getLastPoint() const { return lastPoint_; }

//////////////////////////////////////////////////////////////////////////////////////////////
PointXYZINormal& OctreeContainerLastPoint::getLastPoint() { return lastPoint_; }

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerLastPoint::resetLastPoint() { OctreeContainerLastPoint::reset(); }

}  // namespace jpcc::octree