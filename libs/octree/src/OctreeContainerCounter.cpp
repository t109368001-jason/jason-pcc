#include <jpcc/octree/OctreeContainerCounter.h>

namespace jpcc::octree {

OctreeContainerCounter::OctreeContainerCounter() : OctreeContainerBase(), count_(0) {}

void OctreeContainerCounter::reset() { count_ = 0; }

void OctreeContainerCounter::addPointIndex(const pcl::index_t& index) { count_++; }

}  // namespace jpcc::octree