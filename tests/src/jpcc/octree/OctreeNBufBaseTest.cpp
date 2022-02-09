#include <gtest/gtest.h>

#include "jpcc/octree/OctreeNBufBase.h"

#include "test_data/octree/TestOctree.h"

namespace jpcc::octree {

using namespace pcl::octree;

TEST(OctreeNBufBaseTest, getBranchBitPattern) {
  // given
  const OctreePointCloud<pcl::PointXYZ, OctreeContainerPointIndices, OctreeContainerEmpty,
                         OctreeNBufBase<BUFFER_SIZE, OctreeContainerPointIndices, OctreeContainerEmpty>>& octree =
      getTestOctree();
  // then
  EXPECT_EQ(octree.getBufferSize(), BUFFER_SIZE);
  EXPECT_EQ(octree.getTreeDepth(), 1);

  for (BufferSize b = 0; b < BUFFER_SIZE; ++b) {
    const ChildrenPattern& childrenPattern = octree.getChildrenPattern(*octree.root_node_, b);
    EXPECT_EQ(childrenPattern.to_string(), getTestChildrenPattern(b).to_string());
  }
}

TEST(OctreeNBufBaseTest, getBranchBufferPattern) {
  // given
  const OctreePointCloud<pcl::PointXYZ, OctreeContainerPointIndices, OctreeContainerEmpty,
                         OctreeNBufBase<BUFFER_SIZE, OctreeContainerPointIndices, OctreeContainerEmpty>>& octree =
      getTestOctree();
  // then
  for (ChildrenIndex i = 0; i < 8; i++) {
    const OctreeNBufBase<BUFFER_SIZE, OctreeContainerPointIndices, OctreeContainerEmpty>::BufferPattern bufferPattern =
        octree.getBufferPattern(*octree.root_node_, i);
    for (BufferSize b = 0; b < octree.getBufferSize(); b++) {
      EXPECT_EQ(bufferPattern[b], getTestChildrenPattern(b)[i]);
    }
  }
}

}  // namespace jpcc::octree
