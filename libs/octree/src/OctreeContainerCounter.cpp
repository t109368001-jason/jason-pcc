#include <jpcc/octree/OctreeContainerCounter.h>

namespace jpcc::octree {

OctreeContainerCounter::OctreeContainerCounter() : OctreeContainerBase(), count_(0) { OctreeContainerCounter::reset(); }

void OctreeContainerCounter::reset() { count_ = 0; }

void OctreeContainerCounter::addPointIndex(const Index& index) { count_++; }

size_t OctreeContainerCounter::getCount() const { return count_; }

}  // namespace jpcc::octree