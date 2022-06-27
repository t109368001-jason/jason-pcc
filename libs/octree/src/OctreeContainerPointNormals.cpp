#include <jpcc/octree/OctreeContainerPointNormals.h>

using namespace std;

namespace jpcc::octree {

//////////////////////////////////////////////////////////////////////////////////////////////
OctreeContainerPointNormals::OctreeContainerPointNormals() : azimuths_(), zeniths_() {}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerPointNormals::reset() {
  azimuths_.clear();
  zeniths_.clear();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerPointNormals::addPoint(const PointXYZINormal& point) {
  if (isnan(point.normal_x)) { return; }
  double azimuth;
  if (point.normal_x > 0) {
    azimuth = atan2(point.normal_y, point.normal_x);
  } else if (point.normal_x < 0 && point.normal_y >= 0) {
    azimuth = atan2(point.normal_y, point.normal_x) + M_PI;
  } else if (point.normal_x < 0 && point.normal_y < 0) {
    azimuth = atan2(point.normal_y, point.normal_x) - M_PI;
  } else if (point.normal_x == 0 && point.normal_y > 0) {
    azimuth = M_PI_2;
  } else if (point.normal_x == 0 && point.normal_y < 0) {
    azimuth = -M_PI_2;
  } else {
    azimuth = -0;
  }
  double zenith = acos(point.normal_z);

  assert(!isnan(azimuth));
  assert(!isnan(zenith));

  if (azimuth < 0) { azimuth += M_PI + M_PI; }

  azimuths_.push_back(azimuth);
  zeniths_.push_back(zenith);
}

//////////////////////////////////////////////////////////////////////////////////////////////
const vector<double>& OctreeContainerPointNormals::getAzimuths() const { return azimuths_; }

//////////////////////////////////////////////////////////////////////////////////////////////
const vector<double>& OctreeContainerPointNormals::getZeniths() const { return zeniths_; }

}  // namespace jpcc::octree