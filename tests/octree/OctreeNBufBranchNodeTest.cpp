#include <gtest/gtest.h>

#include <jpcc/octree/OctreeNBufBranchNode.h>

using namespace jpcc::octree;

TEST(OctreeNBufBranchNodeTest, newBySize) {
  // when
  OctreeNBufBranchNode<11> node;
  // then
  EXPECT_EQ(node.getBufferSize(), 11);
  for (uint8_t b = 0; b < 11; ++b) {
    for (uint8_t i = 0; i < 8; ++i) { EXPECT_NO_THROW(node.getChildPtr(b, i)); }
  }
}

TEST(OctreeNBufBranchNodeTest, newByAnother) {
  // given
  OctreeNBufBranchNode<11> node_;
  // when
  OctreeNBufBranchNode<11> node(node_);
  // then
  EXPECT_EQ(node.getBufferSize(), 11);
  for (uint8_t b = 0; b < 11; ++b) {
    for (uint8_t i = 0; i < 8; ++i) { EXPECT_NO_THROW(node.getChildPtr(b, i)); }
  }
}