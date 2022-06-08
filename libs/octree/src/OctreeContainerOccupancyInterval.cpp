#include <jpcc/octree/OctreeContainerOccupancyInterval.h>

using namespace std;
using namespace Eigen;

namespace jpcc::octree {

//////////////////////////////////////////////////////////////////////////////////////////////
OctreeContainerOccupancyInterval::OctreeContainerOccupancyInterval() : count_(0), occupancyIntervals_() {}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerOccupancyInterval::reset() {}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerOccupancyInterval::addPointIndex(const index_t& index) { appendCount(); }

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerOccupancyInterval::appendCount() {
  occupancyIntervals_.conservativeResize(occupancyIntervals_.rows() + 1, NoChange);
  occupancyIntervals_(occupancyIntervals_.rows() - 1, NoChange) = count_;

  count_ = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXi OctreeContainerOccupancyInterval::getOccupancyIntervals() const { return occupancyIntervals_; }

//////////////////////////////////////////////////////////////////////////////////////////////
OctreeContainerOccupancyInterval& OctreeContainerOccupancyInterval::operator++() {
  count_++;
  return *this;
}

}  // namespace jpcc::octree