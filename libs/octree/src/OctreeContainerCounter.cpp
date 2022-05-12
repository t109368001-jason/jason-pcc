#include <jpcc/octree/OctreeContainerCounter.h>

namespace jpcc::octree {

OctreeContainerCounter::OctreeContainerCounter() : OctreeContainerBase(), count_(0) {}

void OctreeContainerCounter::reset() { count_ = 0; }

void OctreeContainerCounter::addPointIndex(const index_t& index) { count_++; }

size_t OctreeContainerCounter::getCount() const { return count_; }

}  // namespace jpcc::octree