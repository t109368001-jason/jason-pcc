#include <jpcc/octree/OctreeContainerGMMWithAdaptivePoint.h>

namespace jpcc::octree {

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