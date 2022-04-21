#include <gtest/gtest.h>

#include <jpcc/common/Common.h>
#include <jpcc/octree/OctreeNBuf.h>

#include "test_data/octree/TestOctree.h"

namespace jpcc::octree {

using namespace pcl::octree;

TEST(OctreeNBufTest, getChildPattern) {
  // given
  OctreePointCloud<Point, OctreeContainerPointIndices, OctreeContainerEmpty, OctreeNBuf<BUFFER_SIZE>> octree =
      getTestOctree();
  // then
  EXPECT_EQ(octree.getBufferSize(), BUFFER_SIZE);
  EXPECT_EQ(octree.getTreeDepth(), 1);

  for (BufferIndex bufferIndex = 0; bufferIndex < BUFFER_SIZE; ++bufferIndex) {
    octree.switchBuffers(bufferIndex);
    const ChildPattern& childPattern =
        dynamic_cast<const OctreeNBuf<BUFFER_SIZE>::BranchNode*>(octree.getRootNode())->getChildPattern(bufferIndex);
    const ChildPattern& currentChildPattern =
        dynamic_cast<const OctreeNBuf<BUFFER_SIZE>::BranchNode*>(octree.getRootNode())
            ->getChildPattern(octree.getBufferIndex());
    EXPECT_EQ(childPattern.to_string(), getTestChildPattern(bufferIndex).to_string());
    EXPECT_EQ(childPattern, currentChildPattern);
  }
}

TEST(OctreeNBufTest, getBufferPattern) {
  // given
  const OctreePointCloud<Point, OctreeContainerPointIndices, OctreeContainerEmpty, OctreeNBuf<BUFFER_SIZE>>& octree =
      getTestOctree();
  // then
  for (ChildIndex childIndex = 0; childIndex < 8; childIndex++) {
    const OctreeNBuf<BUFFER_SIZE>::BufferPattern bufferPattern =
        dynamic_cast<const OctreeNBuf<BUFFER_SIZE>::BranchNode*>(octree.getRootNode())->getBufferPattern(childIndex);
    for (BufferIndex bufferIndex = 0; bufferIndex < octree.getBufferSize(); bufferIndex++) {
      EXPECT_EQ(bufferPattern.test(bufferIndex), getTestChildPattern(bufferIndex)[childIndex]);
    }
  }
}

TEST(OctreeNBufTest, getIndicesByFilter) {
  // given
  OctreePointCloud<Point, OctreeContainerPointIndices, OctreeContainerEmpty, OctreeNBuf<BUFFER_SIZE>> octree =
      getTestOctree();
  auto filter = [](BufferIndex b, const OctreeNBuf<BUFFER_SIZE>::BufferPattern& bufferPattern) {
    return bufferPattern.test(b);
  };
  // then
  for (BufferIndex bufferIndex = 0; bufferIndex < octree.getBufferSize(); bufferIndex++) {
    Indices indices;
    octree.switchBuffers(bufferIndex);
    octree.getIndicesByFilter(
        [filter, bufferIndex](auto&& bufferPattern) {
          return filter(bufferIndex, std::forward<decltype(bufferPattern)>(bufferPattern));
        },
        indices);
    EXPECT_EQ(indices.size(), getTestChildPattern(bufferIndex).count());
  }
}

TEST(OctreeNBufTest, deleteBuffer) {
  // given
  OctreePointCloud<Point, OctreeContainerPointIndices, OctreeContainerEmpty, OctreeNBuf<BUFFER_SIZE>> octree =
      getTestOctree();

  // then
  for (BufferIndex bufferIndex = 0; bufferIndex < octree.getBufferSize(); bufferIndex++) {
    Indices indices;
    octree.deleteBuffer(bufferIndex);
    EXPECT_EQ(octree.getLeafCount(bufferIndex), 0);
    EXPECT_EQ(octree.getBranchCount(bufferIndex), 1);
  }
}

TEST(OctreeNBufTest, reuseBuffer) {
  // given
  OctreePointCloud<Point, OctreeContainerPointIndices, OctreeContainerEmpty, OctreeNBuf<BUFFER_SIZE>> octree =
      getTestOctree();
  auto filter = [](BufferIndex b, const OctreeNBuf<BUFFER_SIZE>::BufferPattern& bufferPattern) {
    return bufferPattern.test(b);
  };
  // then
  for (BufferIndex bufferIndex = 0; bufferIndex < octree.getBufferSize(); bufferIndex++) {
    octree.switchBuffers(bufferIndex);
    {
      Indices indices;
      octree.getIndicesByFilter(
          [filter, bufferIndex](auto&& bufferPattern) {
            return filter(bufferIndex, std::forward<decltype(bufferPattern)>(bufferPattern));
          },
          indices);
      EXPECT_EQ(indices.size(), getTestChildPattern(bufferIndex).count());
    }
    {
      Indices indices;
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
