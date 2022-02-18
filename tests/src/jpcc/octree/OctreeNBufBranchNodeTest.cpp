#include <gtest/gtest.h>

#include "jpcc/octree/OctreeNBufBranchNode.h"

#include "test_data/octree/TestCloud.h"

namespace jpcc::octree {

TEST(OctreeNBufBranchNodeTest, newBySize) {
  // when
  OctreeNBufBranchNode<BUFFER_SIZE> node;
  // then
  EXPECT_EQ(node.getBufferSize(), BUFFER_SIZE);
  for (BufferIndex bufferIndex = 0; bufferIndex < BUFFER_SIZE; bufferIndex++) {
    for (ChildIndex childIndex = 0; childIndex < 8; ++childIndex) {
      OctreeNBufBranchNode<BUFFER_SIZE>::OctreeNode* childNode = node.getChildPtr(bufferIndex, childIndex);
      EXPECT_EQ(childNode, nullptr);
    }
  }
}

TEST(OctreeNBufBranchNodeTest, newByAnother) {
  // given
  OctreeNBufBranchNode<BUFFER_SIZE> node_;
  // when
  OctreeNBufBranchNode<BUFFER_SIZE> node(node_);
  // then
  EXPECT_EQ(node.getBufferSize(), BUFFER_SIZE);
  for (BufferIndex bufferIndex = 0; bufferIndex < BUFFER_SIZE; ++bufferIndex) {
    for (ChildIndex childIndex = 0; childIndex < 8; ++childIndex) {
      OctreeNBufBranchNode<BUFFER_SIZE>::OctreeNode* childNode = node.getChildPtr(bufferIndex, childIndex);
      EXPECT_EQ(childNode, nullptr);
    }
  }
}

}  // namespace jpcc::octree
