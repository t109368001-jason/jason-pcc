#include <gtest/gtest.h>

#include <jpcc/common/Common.h>
#include <jpcc/octree/OctreeNBuf.h>

#include "test_data/octree/TestOctree.h"

namespace jpcc::octree {

using namespace pcl::octree;

TEST(OctreeNBufTest, getChildPattern) {
  // given
  JPCCOctreePointCloud<PointXYZINormal, OctreeContainerPointIndices, OctreeContainerEmpty, OctreeNBuf<BUFFER_SIZE>>
      octreePointCloud = getTestOctree();
  // then
  EXPECT_EQ(octreePointCloud.getBufferSize(), BUFFER_SIZE);
  EXPECT_EQ(octreePointCloud.getTreeDepth(), 1);

  for (BufferIndex bufferIndex = 0; bufferIndex < BUFFER_SIZE; ++bufferIndex) {
    octreePointCloud.switchBuffers(bufferIndex);
    const ChildPattern& childPattern =
        dynamic_cast<const OctreeNBuf<BUFFER_SIZE>::BranchNode*>(octreePointCloud.getRootNode())
            ->getChildPattern(bufferIndex);
    const ChildPattern& currentChildPattern =
        dynamic_cast<const OctreeNBuf<BUFFER_SIZE>::BranchNode*>(octreePointCloud.getRootNode())
            ->getChildPattern(octreePointCloud.getBufferIndex());
    EXPECT_EQ(childPattern.to_string(), getTestChildPattern(bufferIndex).to_string());
    EXPECT_EQ(childPattern, currentChildPattern);
  }
}

TEST(OctreeNBufTest, getBufferPattern) {
  // given
  const JPCCOctreePointCloud<PointXYZINormal, OctreeContainerPointIndices, OctreeContainerEmpty,
                             OctreeNBuf<BUFFER_SIZE>>& octreePointCloud = getTestOctree();
  // then
  for (ChildIndex childIndex = 0; childIndex < 8; childIndex++) {
    const OctreeNBuf<BUFFER_SIZE>::BufferPattern bufferPattern =
        dynamic_cast<const OctreeNBuf<BUFFER_SIZE>::BranchNode*>(octreePointCloud.getRootNode())
            ->getBufferPattern(childIndex);
    for (BufferIndex bufferIndex = 0; bufferIndex < octreePointCloud.getBufferSize(); bufferIndex++) {
      EXPECT_EQ(bufferPattern.test(bufferIndex), getTestChildPattern(bufferIndex)[childIndex]);
    }
  }
}

TEST(OctreeNBufTest, getIndicesByFilter) {
  // given
  JPCCOctreePointCloud<PointXYZINormal, OctreeContainerPointIndices, OctreeContainerEmpty, OctreeNBuf<BUFFER_SIZE>>
       octreePointCloud = getTestOctree();
  auto filter           = [](BufferIndex b, const OctreeNBuf<BUFFER_SIZE>::BufferPattern& bufferPattern) {
    return bufferPattern.test(b);
  };
  // then
  for (BufferIndex bufferIndex = 0; bufferIndex < octreePointCloud.getBufferSize(); bufferIndex++) {
    Indices indices;
    octreePointCloud.switchBuffers(bufferIndex);
    octreePointCloud.getIndicesByFilter(
        [filter, bufferIndex](auto&& bufferPattern) {
          return filter(bufferIndex, std::forward<decltype(bufferPattern)>(bufferPattern));
        },
        indices);
    EXPECT_EQ(indices.size(), getTestChildPattern(bufferIndex).count());
  }
}

TEST(OctreeNBufTest, deleteBuffer) {
  // given
  JPCCOctreePointCloud<PointXYZINormal, OctreeContainerPointIndices, OctreeContainerEmpty, OctreeNBuf<BUFFER_SIZE>>
      octreePointCloud = getTestOctree();

  // then
  for (BufferIndex bufferIndex = 0; bufferIndex < octreePointCloud.getBufferSize(); bufferIndex++) {
    Indices indices;
    octreePointCloud.deleteBuffer(bufferIndex);
    EXPECT_EQ(octreePointCloud.getLeafCount(bufferIndex), 0);
    EXPECT_EQ(octreePointCloud.getBranchCount(bufferIndex), 1);
  }
}

TEST(OctreeNBufTest, reuseBuffer) {
  // given
  JPCCOctreePointCloud<PointXYZINormal, OctreeContainerPointIndices, OctreeContainerEmpty, OctreeNBuf<BUFFER_SIZE>>
       octreePointCloud = getTestOctree();
  auto filter           = [](BufferIndex b, const OctreeNBuf<BUFFER_SIZE>::BufferPattern& bufferPattern) {
    return bufferPattern.test(b);
  };
  // then
  for (BufferIndex bufferIndex = 0; bufferIndex < octreePointCloud.getBufferSize(); bufferIndex++) {
    octreePointCloud.switchBuffers(bufferIndex);
    {
      Indices indices;
      octreePointCloud.getIndicesByFilter(
          [filter, bufferIndex](auto&& bufferPattern) {
            return filter(bufferIndex, std::forward<decltype(bufferPattern)>(bufferPattern));
          },
          indices);
      EXPECT_EQ(indices.size(), getTestChildPattern(bufferIndex).count());
    }
    {
      Indices indices;
      octreePointCloud.setFrame(bufferIndex, getTestCloud(0));
      octreePointCloud.getIndicesByFilter(
          [filter, bufferIndex](auto&& bufferPattern) {
            return filter(bufferIndex, std::forward<decltype(bufferPattern)>(bufferPattern));
          },
          indices);
      EXPECT_EQ(indices.size(), getTestChildPattern(0).count());
    }
  }
}

}  // namespace jpcc::octree
