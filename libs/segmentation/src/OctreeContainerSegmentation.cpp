#include <jpcc/segmentation/OctreeContainerSegmentation.h>

namespace jpcc::octree {

//////////////////////////////////////////////////////////////////////////////////////////////
OctreeContainerSegmentation::OctreeContainerSegmentation() :
    OctreeContainerGMM(), OctreeContainerAdaptivePoint() {}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerSegmentation::reset() {
  OctreeContainerGMM::reset();
  OctreeContainerAdaptivePoint::reset();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerSegmentation::addPoint(const PointXYZINormal& point) {
  OctreeContainerGMM::addPoint(point);
  OctreeContainerAdaptivePoint::addPoint(point);
}

}  // namespace jpcc::octree