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

  for (BufferIndex bufferIndex = 0; bufferIndex < BUFFER_SIZE; ++bufferIndex) {
    const ChildPattern& childPattern = octree.getChildPattern(*octree.root_node_, bufferIndex);
    EXPECT_EQ(childPattern.to_string(), getTestChildPattern(bufferIndex).to_string());
  }
}

TEST(OctreeNBufBaseTest, getBranchBufferPattern) {
  // given
  const OctreePointCloud<pcl::PointXYZ, OctreeContainerPointIndices, OctreeContainerEmpty,
                         OctreeNBufBase<BUFFER_SIZE, OctreeContainerPointIndices, OctreeContainerEmpty>>& octree =
      getTestOctree();
  // then
  for (ChildIndex childIndex = 0; childIndex < 8; childIndex++) {
    const OctreeNBufBase<BUFFER_SIZE, OctreeContainerPointIndices, OctreeContainerEmpty>::BufferPattern bufferPattern =
        octree.getBufferPattern(*octree.root_node_, childIndex);
    for (BufferIndex bufferIndex = 0; bufferIndex < octree.getBufferSize(); bufferIndex++) {
      EXPECT_EQ(bufferPattern[bufferIndex], getTestChildPattern(bufferIndex)[childIndex]);
    }
  }
}

TEST(OctreeNBufBaseTest, getIndicesByFilter) {
  // given
  OctreePointCloud<pcl::PointXYZ, OctreeContainerPointIndices, OctreeContainerEmpty,
                   OctreeNBufBase<BUFFER_SIZE, OctreeContainerPointIndices, OctreeContainerEmpty>>
       octree = getTestOctree();
  auto filter = [](BufferIndex b,
                   const OctreeNBufBase<BUFFER_SIZE, OctreeContainerPointIndices, OctreeContainerEmpty>::BufferPattern&
                       bufferPattern) { return bufferPattern.test(b); };
  // then
  for (BufferIndex bufferIndex = 0; bufferIndex < octree.getBufferSize(); bufferIndex++) {
    pcl::Indices indices;
    octree.switchBuffers(bufferIndex);
    octree.getIndicesByFilter(
        [filter, bufferIndex](auto&& bufferPattern) {
          return filter(bufferIndex, std::forward<decltype(bufferPattern)>(bufferPattern));
        },
        indices);
    EXPECT_EQ(indices.size(), getTestChildPattern(bufferIndex).count());
  }
}

}  // namespace jpcc::octree
