#pragma once

#include <test_data/octree/TestCloud.h>

#include <jpcc/octree/JPCCOctreePointCloud.h>

namespace jpcc::octree {

#define RESOLUTION 0.5

jpcc::octree::JPCCOctreePointCloud<
    pcl::PointXYZ,
    pcl::octree::OctreeContainerPointIndices,
    pcl::octree::OctreeContainerEmpty,
    OctreeNBuf<BUFFER_SIZE, pcl::octree::OctreeContainerPointIndices, pcl::octree::OctreeContainerEmpty>>
getTestOctree();

}  // namespace jpcc::octree
