#include <jpcc/octree/OctreeContainerOccupancyChangeCount.h>

using namespace std;

namespace jpcc::octree {

//////////////////////////////////////////////////////////////////////////////////////////////
OctreeContainerOccupancyChangeCount::OctreeContainerOccupancyChangeCount() :
    OctreeContainerBase(), previous_(false), current_(false), count_(0) {
  OctreeContainerOccupancyChangeCount::reset();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerOccupancyChangeCount::reset() {
  previous_ = false;
  current_  = false;
  count_    = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerOccupancyChangeCount::addPointIndex(const Index& index) {
  current_ = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerOccupancyChangeCount::resetCurrent() {
  current_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerOccupancyChangeCount::compute() {
  if (previous_ == current_) {
    return;
  }

  count_++;
  previous_ = current_;
}

//////////////////////////////////////////////////////////////////////////////////////////////
size_t OctreeContainerOccupancyChangeCount::getCount() const {
  return count_;
}

}  // namespace jpcc::octree