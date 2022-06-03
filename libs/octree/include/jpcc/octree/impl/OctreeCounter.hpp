#pragma once

#include <pcl/octree/octree_base.h>

#include <jpcc/octree/OctreeNBuf.h>

namespace jpcc::octree {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, BufferIndex BUFFER_SIZE>
OctreeCounter<PointT, BUFFER_SIZE>::OctreeCounter(const double resolution) : JPCCOctreePointCloud(resolution) {
  this->defineBoundingBox(resolution * 2);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, BufferIndex BUFFER_SIZE>
typename OctreeCounter<PointT, BUFFER_SIZE>::CountMap
OctreeCounter<PointT, BUFFER_SIZE>::getOccupancyCountToVoxelCount() {
  if constexpr (std::is_same_v<CountMap, std::map<size_t, size_t>>) {
    std::map<size_t, size_t> countMap;
    for (auto it = this->leaf_depth_begin(), end = this->leaf_depth_end(); it != end; ++it) {
      const size_t count = it.getLeafContainer().getCount();
      countMap.try_emplace(count, 0);
      countMap.at(count) = countMap.at(count) + 1;
    }
    return countMap;
  } else if constexpr (std::is_same_v<CountMap, std::map<size_t, std::array<size_t, BUFFER_SIZE>>>) {
    std::map<size_t, std::array<size_t, BUFFER_SIZE>> countArrayMap;
    for (BufferIndex bufferIndex = 0; bufferIndex < BUFFER_SIZE; bufferIndex++) {
      this->switchBuffers(bufferIndex);
      for (auto it = this->leaf_depth_begin(), end = this->leaf_depth_end(); it != end; ++it) {
        const size_t count = it.getLeafContainer().getCount();
        countArrayMap.try_emplace(count, std::array<size_t, BUFFER_SIZE>{0, 0, 0});
        countArrayMap.at(count).at(bufferIndex) = countArrayMap.at(count).at(bufferIndex) + 1;
      }
    }
    return countArrayMap;
  } else {
    static_assert(false, "invalid template type");
  }
}

}  // namespace jpcc::octree