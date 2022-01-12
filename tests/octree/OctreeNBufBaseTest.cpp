#include <gtest/gtest.h>

#define PCL_NO_PRECOMPILE
#include <pcl/octree/octree_pointcloud.h>

#include <jpcc/octree/OctreeNBufBase.h>

using namespace pcl;
using namespace octree;
using namespace jpcc::octree;

#define BUFFER_SIZE 11

TEST(OctreeNBufBaseTest, switchBuffers_and_getBranchBitPattern) {
  // given
  PointCloud<PointXYZ>::Ptr cloud1(new PointCloud<PointXYZ>());
  cloud1->points.push_back(PointXYZ(0.5, 0.5, 0.5));
  // cloud1->points.push_back(PointXYZ(0.5, 0.5, 0.0));
  cloud1->points.push_back(PointXYZ(0.5, 0.0, 0.5));
  // cloud1->points.push_back(PointXYZ(0.5, 0.0, 0.0));
  cloud1->points.push_back(PointXYZ(0.0, 0.5, 0.5));
  // cloud1->points.push_back(PointXYZ(0.0, 0.5, 0.0));
  cloud1->points.push_back(PointXYZ(0.0, 0.0, 0.5));
  // cloud1->points.push_back(PointXYZ(0.0, 0.0, 0.0));
  PointCloud<PointXYZ>::Ptr cloud2(new PointCloud<PointXYZ>());
  cloud2->points.push_back(PointXYZ(0.5, 0.5, 0.5));
  cloud2->points.push_back(PointXYZ(0.5, 0.5, 0.0));
  // cloud2->points.push_back(PointXYZ(0.5, 0.0, 0.5));
  cloud2->points.push_back(PointXYZ(0.5, 0.0, 0.0));
  cloud2->points.push_back(PointXYZ(0.0, 0.5, 0.5));
  // cloud2->points.push_back(PointXYZ(0.0, 0.5, 0.0));
  cloud2->points.push_back(PointXYZ(0.0, 0.0, 0.5));
  cloud2->points.push_back(PointXYZ(0.0, 0.0, 0.0));
  // when
  OctreePointCloud<PointXYZ, OctreeContainerPointIndices, OctreeContainerEmpty,
                   OctreeNBufBase<BUFFER_SIZE, OctreeContainerPointIndices, OctreeContainerEmpty>>
      octree(0.5);
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
