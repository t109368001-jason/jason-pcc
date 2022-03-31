#include <gtest/gtest.h>

#include <jpcc/common/Common.h>
#include <jpcc/octree/OctreeNBufBase.h>

#include "test_data/octree/TestOctree.h"

namespace jpcc::octree {

using namespace pcl::octree;

TEST(OctreeNBufBaseTest, getChildPattern) {
  // given
  OctreePointCloud<Point, OctreeContainerPointIndices, OctreeContainerEmpty, OctreeNBufBase<BUFFER_SIZE>> octree =
      getTestOctree();
  // then
  EXPECT_EQ(octree.getBufferSize(), BUFFER_SIZE);
  EXPECT_EQ(octree.getTreeDepth(), 1);

  for (BufferIndex bufferIndex = 0; bufferIndex < BUFFER_SIZE; ++bufferIndex) {
    octree.switchBuffers(bufferIndex);
    const ChildPattern& childPattern        = octree.getChildPattern(*octree.root_node_, bufferIndex);
    const ChildPattern& currentChildPattern = octree.getChildPattern(*octree.root_node_);
    EXPECT_EQ(childPattern.to_string(), getTestChildPattern(bufferIndex).to_string());
    EXPECT_EQ(childPattern, currentChildPattern);
  }
}

TEST(OctreeNBufBaseTest, getBufferPattern) {
  // given
  const OctreePointCloud<Point, OctreeContainerPointIndices, OctreeContainerEmpty, OctreeNBufBase<BUFFER_SIZE>>&
      octree = getTestOctree();
  // then
  for (ChildIndex childIndex = 0; childIndex < 8; childIndex++) {
    const OctreeNBufBase<BUFFER_SIZE>::BufferPattern bufferPattern =
        octree.getBufferPattern(*octree.root_node_, childIndex);
    for (BufferIndex bufferIndex = 0; bufferIndex < octree.getBufferSize(); bufferIndex++) {
      EXPECT_EQ(bufferPattern.test(bufferIndex), getTestChildPattern(bufferIndex)[childIndex]);
    }
  }
}

TEST(OctreeNBufBaseTest, getIndicesByFilter) {
  // given
  OctreePointCloud<Point, OctreeContainerPointIndices, OctreeContainerEmpty, OctreeNBufBase<BUFFER_SIZE>> octree =
      getTestOctree();
  auto filter = [](BufferIndex b, const OctreeNBufBase<BUFFER_SIZE>::BufferPattern& bufferPattern) {
    return bufferPattern.test(b);
  };
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

TEST(OctreeNBufBaseTest, deleteBuffer) {
  // given
  OctreePointCloud<Point, OctreeContainerPointIndices, OctreeContainerEmpty, OctreeNBufBase<BUFFER_SIZE>> octree =
      getTestOctree();

  // then
  for (BufferIndex bufferIndex = 0; bufferIndex < octree.getBufferSize(); bufferIndex++) {
    pcl::Indices indices;
    octree.deleteBuffer(bufferIndex);
    EXPECT_EQ(octree.leaf_counts_.at(bufferIndex), 0);
    EXPECT_EQ(octree.branch_counts_.at(bufferIndex), 1);
  }
}

TEST(OctreeNBufBaseTest, reuseBuffer) {
  // given
  OctreePointCloud<Point, OctreeContainerPointIndices, OctreeContainerEmpty, OctreeNBufBase<BUFFER_SIZE>> octree =
      getTestOctree();
  auto filter = [](BufferIndex b, const OctreeNBufBase<BUFFER_SIZE>::BufferPattern& bufferPattern) {
    return bufferPattern.test(b);
  };
  // then
  for (BufferIndex bufferIndex = 0; bufferIndex < octree.getBufferSize(); bufferIndex++) {
    octree.switchBuffers(bufferIndex);
    {
      pcl::Indices indices;
      octree.getIndicesByFilter(
          [filter, bufferIndex](auto&& bufferPattern) {
            return filter(bufferIndex, std::forward<decltype(bufferPattern)>(bufferPattern));
          },
          indices);
      EXPECT_EQ(indices.size(), getTestChildPattern(bufferIndex).count());
    }
    {
      pcl::Indices indices;
      octree.deleteBuffer(bufferIndex);
      octree.setInputCloud(getTestCloud(0));
      octree.addPointsFromInputCloud();
      octree.getIndicesByFilter(
          [filter, bufferIndex](auto&& bufferPattern) {
            return filter(bufferIndex, std::forward<decltype(bufferPattern)>(bufferPattern));
          },
          indices);
      EXPECT_EQ(indices.size(), getTestChildPattern(0).count());
    }
  }
}

}  // namespace jpcc::octree
