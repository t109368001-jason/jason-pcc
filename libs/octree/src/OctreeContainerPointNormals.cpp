#include <jpcc/octree/OctreeContainerPointNormals.h>

using namespace std;
using namespace Eigen;

namespace jpcc::octree {

//////////////////////////////////////////////////////////////////////////////////////////////
OctreeContainerPointNormals::OctreeContainerPointNormals() : angles_() {}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerPointNormals::reset() { angles_.conservativeResize(0, NoChange); }

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerPointNormals::addPoint(const PointNormal& point) {
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

  angles_.conservativeResize(angles_.rows() + 1, NoChange);
  angles_(angles_.rows() - 1, AZIMUTH_INDEX) = azimuth;
  angles_(angles_.rows() - 1, ZENITH_INDEX)  = zenith;
}

//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd OctreeContainerPointNormals::getAzimuths() { return angles_.col(AZIMUTH_INDEX); }

//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd OctreeContainerPointNormals::getZeniths() { return angles_.col(ZENITH_INDEX); }

}  // namespace jpcc::octree