#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <jpcc/octree/OctreeNBufBase.h>

#include <test_data/octree/TestData.h>

namespace jpcc::octree {

ChildrenPattern getTestChildrenPattern(BufferSize bufferSelector);

pcl::PointCloud<pcl::PointXYZ>::Ptr getTestCloud(BufferSize bufferSelector);

}  // namespace jpcc::octree
