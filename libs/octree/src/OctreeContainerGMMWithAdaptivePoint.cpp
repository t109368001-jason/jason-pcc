#include <jpcc/octree/OctreeContainerGMMWithAdaptivePoint.h>

namespace jpcc::octree {

//////////////////////////////////////////////////////////////////////////////////////////////
OctreeContainerGMMWithAdaptivePoint::OctreeContainerGMMWithAdaptivePoint() :
    OctreeContainerGMM(), OctreeContainerAdaptivePoint() {}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerGMMWithAdaptivePoint::reset() {
  OctreeContainerGMM::reset();
  OctreeContainerAdaptivePoint::reset();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerGMMWithAdaptivePoint::addPoint(const PointXYZINormal& point) {
  OctreeContainerGMM::addPoint(point);
  if (OctreeContainerGMM::getGMM()) { OctreeContainerAdaptivePoint::addPoint(point); }
}

}  // namespace jpcc::octree