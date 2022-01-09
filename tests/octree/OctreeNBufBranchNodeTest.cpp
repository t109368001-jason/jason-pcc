#include <gtest/gtest.h>

#include <jpcc/octree/OctreeNBufBranchNode.h>

using namespace jpcc::octree;

TEST(OctreeNBufBranchNodeTest, newBySize) {
  // given
  uint8_t n = 11;
  // when
  OctreeNBufBranchNode<> node(n);
  // then
  EXPECT_EQ(node.getBufferSize(), n);
  for (uint8_t b = 0; b < n; ++b) {
    for (uint8_t i = 0; i < 8; ++i) { EXPECT_NO_THROW(node.getChildPtr(b, i)); }
  }
}

TEST(OctreeNBufBranchNodeTest, newByAnother) {
  // given
  uint8_t                n = 11;
  OctreeNBufBranchNode<> node_(n);
  // when
  OctreeNBufBranchNode<> node(node_);
  // then
  EXPECT_EQ(node.getBufferSize(), n);
  for (uint8_t b = 0; b < n; ++b) {
    for (uint8_t i = 0; i < 8; ++i) { EXPECT_NO_THROW(node.getChildPtr(b, i)); }
  }
}