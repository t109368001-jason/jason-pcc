#include <jpcc/octree/OctreeContainerEditableIndex.h>

namespace jpcc::octree {

void OctreeContainerEditableIndex::setPointIndex(Index index) {
  data_ = index;
}

}  // namespace jpcc::octree