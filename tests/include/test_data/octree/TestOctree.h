#pragma once

#define PCL_NO_PRECOMPILE
#include <pcl/octree/octree_pointcloud.h>

#include <test_data/octree/TestCloud.h>

namespace jpcc::octree {

#define RESOLUTION 0.5
#define BOUNDING_BOX 1.0

pcl::octree::OctreePointCloud<
    Point,
    pcl::octree::OctreeContainerPointIndices,
    pcl::octree::OctreeContainerEmpty,
    OctreeNBufBase<BUFFER_SIZE, pcl::octree::OctreeContainerPointIndices, pcl::octree::OctreeContainerEmpty>>
getTestOctree();

}  // namespace jpcc::octree
