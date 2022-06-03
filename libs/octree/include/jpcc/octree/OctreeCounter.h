#pragma once

#include <array>
#include <map>

#include <pcl/octree/octree_base.h>
#include <pcl/octree/octree_container.h>

#include <jpcc/octree/OctreeContainerCounter.h>
#include <jpcc/octree/OctreeNBuf.h>
#include <jpcc/octree/JPCCOctreePointCloud.h>

namespace jpcc::octree {

template <typename PointT, BufferIndex BUFFER_SIZE = 1>
class OctreeCounter
    : public JPCCOctreePointCloud<
          PointT,
          OctreeContainerCounter,
          pcl::octree::OctreeContainerEmpty,
          typename std::conditional<
              BUFFER_SIZE == 1,
              pcl::octree::OctreeBase<OctreeContainerCounter, pcl::octree::OctreeContainerEmpty>,
              OctreeNBuf<BUFFER_SIZE, OctreeContainerCounter, pcl::octree::OctreeContainerEmpty>>::type> {
 public:
  using OctreeBase =
      typename std::conditional_t<BUFFER_SIZE == 1,
                                  pcl::octree::OctreeBase<OctreeContainerCounter, pcl::octree::OctreeContainerEmpty>,
                                  OctreeNBuf<BUFFER_SIZE, OctreeContainerCounter, pcl::octree::OctreeContainerEmpty>>;
  using JPCCOctreePointCloud = JPCCOctreePointCloud<
      PointT,
      OctreeContainerCounter,
      pcl::octree::OctreeContainerEmpty,
      typename std::conditional_t<BUFFER_SIZE == 1,
                                  pcl::octree::OctreeBase<OctreeContainerCounter, pcl::octree::OctreeContainerEmpty>,
                                  OctreeNBuf<BUFFER_SIZE, OctreeContainerCounter, pcl::octree::OctreeContainerEmpty>>>;

  using CountMap = std::conditional_t<
      (BUFFER_SIZE > 0),
      std::conditional_t<BUFFER_SIZE == 1, std::map<size_t, size_t>, std::map<size_t, std::array<size_t, BUFFER_SIZE>>>,
      void>;

  OctreeCounter(double resolution);

  CountMap getOccupancyCountToVoxelCount();
};

}  // namespace jpcc::octree

#include <jpcc/octree/impl/OctreeCounter.hpp>
