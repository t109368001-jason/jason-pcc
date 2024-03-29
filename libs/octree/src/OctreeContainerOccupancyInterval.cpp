#include <jpcc/octree/OctreeContainerOccupancyInterval.h>

using namespace std;

namespace jpcc::octree {

//////////////////////////////////////////////////////////////////////////////////////////////
OctreeContainerOccupancyInterval::OctreeContainerOccupancyInterval() : OctreeContainerBase(), count_(0) {
  OctreeContainerOccupancyInterval::reset();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerOccupancyInterval::reset() {
  count_ = 0;
  occupancyIntervals_.clear();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerOccupancyInterval::addPointIndex(const Index& index) {
  appendCount();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerOccupancyInterval::appendCount() {
  if (count_ != 0) {
    occupancyIntervals_.push_back(count_);
  }

  count_ = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////
const vector<int>& OctreeContainerOccupancyInterval::getOccupancyIntervals() const {
  return occupancyIntervals_;
}

//////////////////////////////////////////////////////////////////////////////////////////////
OctreeContainerOccupancyInterval& OctreeContainerOccupancyInterval::operator++() {
  count_++;
  return *this;
}

}  // namespace jpcc::octree