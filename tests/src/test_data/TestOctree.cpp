#include <test_data/octree/TestOctree.h>

namespace jpcc::octree {

using namespace pcl::octree;

OctreePointCloud<pcl::PointXYZ,
                 OctreeContainerPointIndices,
                 OctreeContainerEmpty,
                 OctreeNBufBase<BUFFER_SIZE, OctreeContainerPointIndices, OctreeContainerEmpty>>
getTestOctree() {
  OctreePointCloud<pcl::PointXYZ, OctreeContainerPointIndices, OctreeContainerEmpty,
                   OctreeNBufBase<BUFFER_SIZE, OctreeContainerPointIndices, OctreeContainerEmpty>>
      octree(RESOLUTION);
  octree.defineBoundingBox(BOUNDING_BOX);
  for (uint8_t b = 0; b < BUFFER_SIZE; ++b) {
    octree.switchBuffers(b);
    octree.setInputCloud(getTestCloud(b));
    octree.addPointsFromInputCloud();
  }
  return octree;
}

}  // namespace jpcc::octree
