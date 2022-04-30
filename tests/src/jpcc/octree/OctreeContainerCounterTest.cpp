#include <gtest/gtest.h>

#define PCL_NO_PRECOMPILE
#include <pcl/octree/octree_pointcloud.h>

#include <jpcc/octree/OctreeContainerCounter.h>

#include <jpcc/common/Common.h>
#include <jpcc/octree/OctreeNBuf.h>

#include "test_data/octree/TestOctree.h"

using namespace std;

namespace jpcc::octree {

using OctreeNuf = OctreeNBuf<1, OctreeContainerCounter, pcl::octree::OctreeContainerEmpty>;
using OctreePointCloud =
    pcl::octree::OctreePointCloud<Point, OctreeContainerCounter, pcl::octree::OctreeContainerEmpty, OctreeNuf>;

TEST(OctreeContainerCounterTest, getChildPattern) {
  OctreePointCloud octree(RESOLUTION);

  octree.defineBoundingBox(BOUNDING_BOX);

  for (BufferIndex bufferIndex = 0; bufferIndex < BUFFER_SIZE; ++bufferIndex) {
    octree.switchBuffers(0);
    octree.setInputCloud(getTestCloud(bufferIndex));
    octree.addPointsFromInputCloud();
  }
  vector<size_t> counts = getCounts();
  for (ChildIndex childIndex = 0; childIndex < 8; ++childIndex) {
    auto rootNode = dynamic_cast<const OctreePointCloud::BranchNode*>(octree.getRootNode());
    auto leafNode = dynamic_cast<const OctreePointCloud::LeafNode*>(rootNode->getChildPtr(0, childIndex));
    EXPECT_EQ(leafNode->getContainer().getCount(), counts.at(childIndex));
  }
}

}  // namespace jpcc::octree