#include <gtest/gtest.h>

#include <jpcc/common/Common.h>
#include <jpcc/octree/OctreeContainerCounter.h>

#include "test_data/octree/TestOctree.h"

#include <jpcc/octree/JPCCOctreePointCloud.h>

using namespace std;
using namespace pcl::octree;

namespace jpcc::octree {

using OctreePointCloudT = JPCCOctreePointCloud<PointXYZINormal,
                                               OctreeContainerCounter,
                                               OctreeContainerEmpty,
                                               OctreeBase<OctreeContainerCounter, OctreeContainerEmpty>>;

TEST(OctreeContainerCounterTest, getChildPattern) {
  OctreePointCloudT octreePointCloud(RESOLUTION);

  for (BufferIndex bufferIndex = 0; bufferIndex < BUFFER_SIZE; ++bufferIndex) {
    octreePointCloud.setFrame(getTestCloud(bufferIndex));
  }
  vector<size_t> counts = getCounts();

  int i = 0;
  for (auto it = octreePointCloud.leaf_breadth_begin(), end = octreePointCloud.leaf_breadth_end(); it != end;
       ++it, i++) {
    EXPECT_EQ(it.getLeafContainer().getCount(), counts.at(i));
  }
}

}  // namespace jpcc::octree