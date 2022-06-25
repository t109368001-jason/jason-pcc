#include <jpcc/octree/OctreeContainerReflectivitys.h>

using namespace std;

namespace jpcc::octree {

//////////////////////////////////////////////////////////////////////////////////////////////
OctreeContainerReflectivitys::OctreeContainerReflectivitys() : reflectivitys_() {}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerReflectivitys::reset() { reflectivitys_.clear(); }

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerReflectivitys::addPoint(const PointXYZINormal& point) {
  reflectivitys_.push_back((Reflectivity)point.data[3]);
}

//////////////////////////////////////////////////////////////////////////////////////////////
const std::vector<Reflectivity>& OctreeContainerReflectivitys::getReflectivitys() const { return reflectivitys_; }

}  // namespace jpcc::octree