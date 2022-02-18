#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <jpcc/octree/OctreeNBufBase.h>

#include <test_data/octree/TestData.h>

namespace jpcc::octree {

ChildPattern getTestChildPattern(BufferIndex bufferSelector);

pcl::PointCloud<pcl::PointXYZ>::Ptr getTestCloud(BufferIndex bufferSelector);

}  // namespace jpcc::octree
