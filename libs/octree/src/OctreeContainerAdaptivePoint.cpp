#include <jpcc/octree/OctreeContainerAdaptivePoint.h>

using namespace std;

namespace jpcc::octree {

//////////////////////////////////////////////////////////////////////////////////////////////
OctreeContainerAdaptivePoint::OctreeContainerAdaptivePoint() : point_() { reset(); }

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerAdaptivePoint::reset() {
  point_.x         = numeric_limits<float>::quiet_NaN();
  point_.y         = numeric_limits<float>::quiet_NaN();
  point_.z         = numeric_limits<float>::quiet_NaN();
  point_.intensity = numeric_limits<float>::quiet_NaN();
  point_.normal_x  = numeric_limits<float>::quiet_NaN();
  point_.normal_y  = numeric_limits<float>::quiet_NaN();
  point_.normal_z  = numeric_limits<float>::quiet_NaN();
  point_.curvature = numeric_limits<float>::quiet_NaN();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerAdaptivePoint::addPoint(const PointXYZINormal& point) {
  assert(!isnan(point.x));
  points_.push_back(point);
}

void OctreeContainerAdaptivePoint::updatePoint(double alpha) {
  if (!points_.empty()) {
    if (isnan(point_.x)) {
      point_.x         = points_.back().x;
      point_.y         = points_.back().y;
      point_.z         = points_.back().z;
      point_.intensity = points_.back().intensity;
      point_.normal_x  = points_.back().normal_x;
      point_.normal_y  = points_.back().normal_y;
      point_.normal_z  = points_.back().normal_z;
      point_.curvature = points_.back().curvature;
      points_.pop_back();
    }
    for (const auto& point : points_) {
      point_.x         = (1.0 - alpha) * point_.x + alpha * point.x;
      point_.y         = (1.0 - alpha) * point_.y + alpha * point.y;
      point_.z         = (1.0 - alpha) * point_.z + alpha * point.z;
      point_.intensity = (1.0 - alpha) * point_.intensity + alpha * point.intensity;
      point_.normal_x  = (1.0 - alpha) * point_.normal_x + alpha * point.normal_x;
      point_.normal_y  = (1.0 - alpha) * point_.normal_y + alpha * point.normal_y;
      point_.normal_z  = (1.0 - alpha) * point_.normal_z + alpha * point.normal_z;
    }
    points_.clear();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
const PointXYZINormal& OctreeContainerAdaptivePoint::getPoint() const { return point_; }

//////////////////////////////////////////////////////////////////////////////////////////////
PointXYZINormal& OctreeContainerAdaptivePoint::getPoint() { return point_; }

}  // namespace jpcc::octree