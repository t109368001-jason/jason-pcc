#include <jpcc/segmentation/OctreeContainerGMMWithAdaptivePoint.h>

namespace jpcc::segmentation {

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
  OctreeContainerAdaptivePoint::addPoint(point);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerGMMWithAdaptivePoint::update(const double alpha) {
  OctreeContainerGMM::addTrainSample();
  OctreeContainerAdaptivePoint::updateAdaptivePoint(alpha);
}

}  // namespace jpcc::octree