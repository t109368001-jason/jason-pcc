#include <test_data/octree/TestOctree.h>

namespace jpcc::octree {

using namespace pcl::octree;

OctreePointCloud<Point, OctreeContainerPointIndices, OctreeContainerEmpty, OctreeNBufBase<BUFFER_SIZE>>
getTestOctree() {
  OctreePointCloud<Point, OctreeContainerPointIndices, OctreeContainerEmpty, OctreeNBufBase<BUFFER_SIZE>> octree(
      RESOLUTION);
  octree.defineBoundingBox(BOUNDING_BOX);
  for (BufferIndex bufferIndex = 0; bufferIndex < BUFFER_SIZE; ++bufferIndex) {
    octree.switchBuffers(bufferIndex);
    octree.setInputCloud(getTestCloud(bufferIndex));
    octree.addPointsFromInputCloud();
  }
  return octree;
}

}  // namespace jpcc::octree
