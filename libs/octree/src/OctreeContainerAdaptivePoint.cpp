#include <jpcc/octree/OctreeContainerAdaptivePoint.h>

using namespace std;

namespace jpcc::octree {

//////////////////////////////////////////////////////////////////////////////////////////////
OctreeContainerAdaptivePoint::OctreeContainerAdaptivePoint() : OctreeContainerLastPoint(), adaptivePoint_() { reset(); }

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerAdaptivePoint::reset() {
  adaptivePoint_.x         = numeric_limits<float>::quiet_NaN();
  adaptivePoint_.y         = numeric_limits<float>::quiet_NaN();
  adaptivePoint_.z         = numeric_limits<float>::quiet_NaN();
  adaptivePoint_.intensity = numeric_limits<float>::quiet_NaN();
  adaptivePoint_.normal_x  = numeric_limits<float>::quiet_NaN();
  adaptivePoint_.normal_y  = numeric_limits<float>::quiet_NaN();
  adaptivePoint_.normal_z  = numeric_limits<float>::quiet_NaN();
  adaptivePoint_.curvature = numeric_limits<float>::quiet_NaN();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerAdaptivePoint::addPoint(const PointXYZINormal& point) { OctreeContainerLastPoint::addPoint(point); }

void OctreeContainerAdaptivePoint::updateAdaptivePoint(double alpha) {
  if (isnan(lastPoint_.x)) { return; }
  if (isnan(adaptivePoint_.x)) {
    adaptivePoint_.x         = lastPoint_.x;
    adaptivePoint_.y         = lastPoint_.y;
    adaptivePoint_.z         = lastPoint_.z;
    adaptivePoint_.intensity = lastPoint_.intensity;
    adaptivePoint_.normal_x  = lastPoint_.normal_x;
    adaptivePoint_.normal_y  = lastPoint_.normal_y;
    adaptivePoint_.normal_z  = lastPoint_.normal_z;
    adaptivePoint_.curvature = lastPoint_.curvature;
  } else {
    adaptivePoint_.x         = (1.0 - alpha) * adaptivePoint_.x + alpha * lastPoint_.x;
    adaptivePoint_.y         = (1.0 - alpha) * adaptivePoint_.y + alpha * lastPoint_.y;
    adaptivePoint_.z         = (1.0 - alpha) * adaptivePoint_.z + alpha * lastPoint_.z;
    adaptivePoint_.intensity = (1.0 - alpha) * adaptivePoint_.intensity + alpha * lastPoint_.intensity;
    adaptivePoint_.normal_x  = (1.0 - alpha) * adaptivePoint_.normal_x + alpha * lastPoint_.normal_x;
    adaptivePoint_.normal_y  = (1.0 - alpha) * adaptivePoint_.normal_y + alpha * lastPoint_.normal_y;
    adaptivePoint_.normal_z  = (1.0 - alpha) * adaptivePoint_.normal_z + alpha * lastPoint_.normal_z;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
const PointXYZINormal& OctreeContainerAdaptivePoint::getAdaptivePoint() const { return adaptivePoint_; }

//////////////////////////////////////////////////////////////////////////////////////////////
PointXYZINormal& OctreeContainerAdaptivePoint::getAdaptivePoint() { return adaptivePoint_; }

}  // namespace jpcc::octree