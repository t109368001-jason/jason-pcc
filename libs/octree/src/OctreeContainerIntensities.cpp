#include <jpcc/octree/OctreeContainerIntensities.h>

using namespace std;

namespace jpcc::octree {

//////////////////////////////////////////////////////////////////////////////////////////////
OctreeContainerIntensities::OctreeContainerIntensities() : OctreeContainerBase() {
  OctreeContainerIntensities::reset();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerIntensities::reset() { intensities_.clear(); }

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerIntensities::addPoint(const PointXYZINormal& point) { intensities_.push_back(point.intensity); }

//////////////////////////////////////////////////////////////////////////////////////////////
const std::vector<float>& OctreeContainerIntensities::getIntensities() const { return intensities_; }

}  // namespace jpcc::octree