#include <test_data/octree/TestOctree.h>

namespace jpcc::octree {

using namespace pcl::octree;

JPCCOctreePointCloud<pcl::PointXYZ, OctreeContainerPointIndices, OctreeContainerEmpty, OctreeNBuf<BUFFER_SIZE>>
getTestOctree() {
  JPCCOctreePointCloud<pcl::PointXYZ, OctreeContainerPointIndices, OctreeContainerEmpty, OctreeNBuf<BUFFER_SIZE>>
      octreePointCloud(RESOLUTION);
  for (BufferIndex bufferIndex = 0; bufferIndex < BUFFER_SIZE; ++bufferIndex) {
    octreePointCloud.setFrame(bufferIndex, getTestCloud(bufferIndex));
  }
  return octreePointCloud;
}

}  // namespace jpcc::octree
