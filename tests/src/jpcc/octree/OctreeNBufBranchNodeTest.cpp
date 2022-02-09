#include <gtest/gtest.h>

#include "jpcc/octree/OctreeNBufBranchNode.h"

#include "test_data/octree/TestCloud.h"

namespace jpcc::octree {

TEST(OctreeNBufBranchNodeTest, newBySize) {
  // when
  OctreeNBufBranchNode<BUFFER_SIZE> node;
  // then
  EXPECT_EQ(node.getBufferSize(), BUFFER_SIZE);
  for (BufferSize b = 0; b < BUFFER_SIZE; b++) {
    for (ChildrenIndex i = 0; i < 8; ++i) {
      OctreeNBufBranchNode<BUFFER_SIZE>::OctreeNode* childPtr = node.getChildPtr(b, i);
      EXPECT_EQ(childPtr, nullptr);
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
  for (BufferSize b = 0; b < BUFFER_SIZE; ++b) {
    for (ChildrenIndex i = 0; i < 8; ++i) {
      OctreeNBufBranchNode<BUFFER_SIZE>::OctreeNode* childPtr = node.getChildPtr(b, i);
      EXPECT_EQ(childPtr, nullptr);
    }
  }
}

}  // namespace jpcc::octree
