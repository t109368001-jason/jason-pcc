#pragma once

#include <test_data/octree/TestCloud.h>

#include <jpcc/octree/OctreePointCloud.h>

namespace jpcc::octree {

#define RESOLUTION 0.5
#define BOUNDING_BOX 1.0

jpcc::octree::OctreePointCloud<
    Point,
    pcl::octree::OctreeContainerPointIndices,
    pcl::octree::OctreeContainerEmpty,
    OctreeNBuf<BUFFER_SIZE, pcl::octree::OctreeContainerPointIndices, pcl::octree::OctreeContainerEmpty>>
getTestOctree();

}  // namespace jpcc::octree
