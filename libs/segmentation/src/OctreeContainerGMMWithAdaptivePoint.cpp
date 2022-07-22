#include <jpcc/segmentation/OctreeContainerGMMWithAdaptivePoint.h>

namespace jpcc::segmentation {

//////////////////////////////////////////////////////////////////////////////////////////////
OctreeContainerGMMWithAdaptivePoint::OctreeContainerGMMWithAdaptivePoint() :
    OctreeContainerStaticFlag(), OctreeContainerGMM(), OctreeContainerAdaptivePoint() {}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerGMMWithAdaptivePoint::reset() {
  OctreeContainerStaticFlag::reset();
  OctreeContainerGMM::reset();
  OctreeContainerAdaptivePoint::reset();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerGMMWithAdaptivePoint::addPoint(const PointXYZINormal& point) {
  OctreeContainerGMM::addPoint(point);
  OctreeContainerAdaptivePoint::addPoint(point);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerGMMWithAdaptivePoint::appendTrainSamples(const double alpha) {
  OctreeContainerGMM::addTrainSample();
  OctreeContainerAdaptivePoint::updateAdaptivePoint(alpha);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerGMMWithAdaptivePoint::updateModel(const double alpha, const double minimumVariance) {
  OctreeContainerGMM::updateModel(alpha, minimumVariance);
  OctreeContainerAdaptivePoint::updateAdaptivePoint(alpha);
}

}  // namespace jpcc::segmentation