#include <gtest/gtest.h>

#define PCL_NO_PRECOMPILE
#include <pcl/octree/octree_pointcloud.h>

#include <jpcc/octree/OctreeNBufBase.h>

using namespace pcl;
using namespace octree;
using namespace jpcc::octree;

#define BUFFER_SIZE 11

using OctreeNBufBaseT = OctreeNBufBase<BUFFER_SIZE, OctreeContainerPointIndices, OctreeContainerEmpty>;

PointCloud<PointXYZ>::Ptr getTestCloud1() {
  PointCloud<PointXYZ>::Ptr cloud1(new PointCloud<PointXYZ>());
  cloud1->points.emplace_back(0.5, 0.5, 0.5);
  // cloud1->points.push_back(PointXYZ(0.5, 0.5, 0.0));
  cloud1->points.emplace_back(PointXYZ(0.5, 0.0, 0.5));
  // cloud1->points.push_back(PointXYZ(0.5, 0.0, 0.0));
  cloud1->points.emplace_back(PointXYZ(0.0, 0.5, 0.5));
  // cloud1->points.push_back(PointXYZ(0.0, 0.5, 0.0));
  cloud1->points.emplace_back(PointXYZ(0.0, 0.0, 0.5));
  // cloud1->points.push_back(PointXYZ(0.0, 0.0, 0.0));
  return cloud1;
}

PointCloud<PointXYZ>::Ptr getTestCloud2() {
  PointCloud<PointXYZ>::Ptr cloud2(new PointCloud<PointXYZ>());
  cloud2->points.emplace_back(PointXYZ(0.5, 0.5, 0.5));
  cloud2->points.emplace_back(PointXYZ(0.5, 0.5, 0.0));
  // cloud2->points.push_back(PointXYZ(0.5, 0.0, 0.5));
  cloud2->points.emplace_back(PointXYZ(0.5, 0.0, 0.0));
  cloud2->points.emplace_back(PointXYZ(0.0, 0.5, 0.5));
  // cloud2->points.push_back(PointXYZ(0.0, 0.5, 0.0));
  cloud2->points.emplace_back(PointXYZ(0.0, 0.0, 0.5));
  cloud2->points.emplace_back(PointXYZ(0.0, 0.0, 0.0));
  return cloud2;
}

auto getTestOctree() {
  const PointCloud<PointXYZ>::Ptr& cloud1 = getTestCloud1();
  const PointCloud<PointXYZ>::Ptr& cloud2 = getTestCloud2();
  OctreePointCloud<PointXYZ, OctreeContainerPointIndices, OctreeContainerEmpty, OctreeNBufBaseT> octree(0.5);
  octree.defineBoundingBox(1.0);
  for (uint8_t b = 0; b < BUFFER_SIZE; ++b) {
    octree.switchBuffers(b);
    if (b % 2 == 0) {
      octree.setInputCloud(cloud1);
    } else {
      octree.setInputCloud(cloud2);
    }
    octree.addPointsFromInputCloud();
  }
  return octree;
}

TEST(OctreeNBufBaseTest, switchBuffers_and_getBranchBitPattern) {
  // given
  const PointCloud<PointXYZ>::Ptr& cloud1 = getTestCloud1();
  const PointCloud<PointXYZ>::Ptr& cloud2 = getTestCloud2();
  // when
  OctreePointCloud<PointXYZ, OctreeContainerPointIndices, OctreeContainerEmpty, OctreeNBufBaseT> octree(0.5);
  octree.defineBoundingBox(1.0);
  for (uint8_t b = 0; b < BUFFER_SIZE; ++b) {
    octree.switchBuffers(b);
    if (b % 2 == 0) {
      octree.setInputCloud(cloud1);
    } else {
      octree.setInputCloud(cloud2);
    }
    octree.addPointsFromInputCloud();
  }
  // then
  EXPECT_EQ(octree.getBufferSize(), BUFFER_SIZE);
  EXPECT_EQ(octree.getTreeDepth(), 1);

  for (uint8_t b = 0; b < BUFFER_SIZE; ++b) {
    uint8_t bitPattern = octree.getBranchBitPattern(*octree.root_node_, b);
    if (b % 2 == 0) {
      EXPECT_EQ(bitPattern, 0b10101010);
    } else {
      EXPECT_EQ(bitPattern, 0b11011011);
    }
  }
}

TEST(OctreeNBufBaseTest, getBranchBufferPattern) {
  const OctreePointCloud<PointXYZ, OctreeContainerPointIndices, OctreeContainerEmpty, OctreeNBufBaseT>& octree =
      getTestOctree();
  const OctreeNBufBaseT::BufferPattern bufferPattern0 = octree.getBranchBufferPattern(*octree.root_node_, 0);
  EXPECT_EQ(bufferPattern0.to_string(), "01010101010");
  const OctreeNBufBaseT::BufferPattern bufferPattern1 = octree.getBranchBufferPattern(*octree.root_node_, 1);
  EXPECT_EQ(bufferPattern1.to_string(), "11111111111");
  const OctreeNBufBaseT::BufferPattern bufferPattern2 = octree.getBranchBufferPattern(*octree.root_node_, 2);
  EXPECT_EQ(bufferPattern2.to_string(), "00000000000");
  const OctreeNBufBaseT::BufferPattern bufferPattern3 = octree.getBranchBufferPattern(*octree.root_node_, 3);
  EXPECT_EQ(bufferPattern3.to_string(), "11111111111");
  const OctreeNBufBaseT::BufferPattern bufferPattern4 = octree.getBranchBufferPattern(*octree.root_node_, 4);
  EXPECT_EQ(bufferPattern4.to_string(), "01010101010");
  const OctreeNBufBaseT::BufferPattern bufferPattern5 = octree.getBranchBufferPattern(*octree.root_node_, 5);
  EXPECT_EQ(bufferPattern5.to_string(), "10101010101");
  const OctreeNBufBaseT::BufferPattern bufferPattern6 = octree.getBranchBufferPattern(*octree.root_node_, 6);
  EXPECT_EQ(bufferPattern6.to_string(), "01010101010");
  const OctreeNBufBaseT::BufferPattern bufferPattern7 = octree.getBranchBufferPattern(*octree.root_node_, 7);
  EXPECT_EQ(bufferPattern7.to_string(), "11111111111");
}
