#include <gtest/gtest.h>

#define PCL_NO_PRECOMPILE
#include <pcl/octree/octree_pointcloud.h>

#include <jpcc/common/Common.h>
#include <jpcc/octree/OctreeContainerCounter.h>

#include "test_data/octree/TestOctree.h"

using namespace std;
using namespace pcl::octree;

namespace jpcc::octree {

using OctreePointCloud = OctreePointCloud<Point,
                                          OctreeContainerCounter,
                                          OctreeContainerEmpty,
                                          OctreeBase<OctreeContainerCounter, OctreeContainerEmpty>>;

TEST(OctreeContainerCounterTest, getChildPattern) {
  OctreePointCloud octree(RESOLUTION);

  octree.defineBoundingBox(BOUNDING_BOX);

  for (BufferIndex bufferIndex = 0; bufferIndex < BUFFER_SIZE; ++bufferIndex) {
    octree.setInputCloud(getTestCloud(bufferIndex));
    octree.addPointsFromInputCloud();
  }
  vector<size_t> counts = getCounts();

  int i = 0;
  for (auto it = octree.leaf_breadth_begin(), end = octree.leaf_breadth_end(); it != end; ++it, i++) {
    EXPECT_EQ(it.getLeafContainer().getCount(), counts.at(i));
  }
}

}  // namespace jpcc::octree