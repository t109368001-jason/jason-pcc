#include <test_data/octree/TestOctree.h>

namespace jpcc::octree {

using namespace pcl::octree;

JPCCOctreePointCloud<Point, OctreeContainerPointIndices, OctreeContainerEmpty, OctreeNBuf<BUFFER_SIZE>>
getTestOctree() {
  JPCCOctreePointCloud<Point, OctreeContainerPointIndices, OctreeContainerEmpty, OctreeNBuf<BUFFER_SIZE>>
      octreePointCloud(RESOLUTION);
  octreePointCloud.defineBoundingBox(BOUNDING_BOX);
  for (BufferIndex bufferIndex = 0; bufferIndex < BUFFER_SIZE; ++bufferIndex) {
    octreePointCloud.switchBuffers(bufferIndex);
    octreePointCloud.setInputCloud(getTestCloud(bufferIndex));
    octreePointCloud.addPointsFromInputCloud();
  }
  return octreePointCloud;
}

}  // namespace jpcc::octree
