#include "VoxelOccupancyChangeCountToVoxelCount.h"

#include <cmath>
#include <map>

#include <Eigen/Dense>

using namespace std;
using namespace std::filesystem;
using namespace jpcc::octree;

namespace jpcc {

//////////////////////////////////////////////////////////////////////////////////////////////
VoxelOccupancyChangeCountToVoxelCount::VoxelOccupancyChangeCountToVoxelCount(const float&       frequency,
                                                                             const double&      resolution,
                                                                             const std::string& outputDir) :
    Analyzer(frequency, resolution, outputDir, "VoxelOccupancyChangeCountToVoxelCount"), octree_(resolution) {}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelOccupancyChangeCountToVoxelCount::compute(FrameConstPtr background,
                                                    FrameConstPtr dynamic,
                                                    FrameConstPtr other) {
  for (BufferIndex bufferIndex = 0; bufferIndex < BUFFER_SIZE; bufferIndex++) {
    octree_.switchBuffers(bufferIndex);
    for (auto it = octree_.leaf_depth_begin(), end = octree_.leaf_depth_end(); it != end; ++it) {
      OctreeContainerOccupancyChangeCount& leafNode = it.getLeafContainer();
      leafNode.resetCurrent();
    }
  }
  octree_.addFrame(0, background);
  octree_.addFrame(1, dynamic);
  octree_.addFrame(2, other);
  for (BufferIndex bufferIndex = 0; bufferIndex < BUFFER_SIZE; bufferIndex++) {
    octree_.switchBuffers(bufferIndex);
    for (auto it = octree_.leaf_depth_begin(), end = octree_.leaf_depth_end(); it != end; ++it) {
      OctreeContainerOccupancyChangeCount& leafNode = it.getLeafContainer();
      leafNode.compute();
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelOccupancyChangeCountToVoxelCount::finalCompute() {
  std::map<size_t, std::array<size_t, BUFFER_SIZE>> countMap;
  for (BufferIndex bufferIndex = 0; bufferIndex < BUFFER_SIZE; bufferIndex++) {
    octree_.switchBuffers(bufferIndex);
    for (auto it = octree_.leaf_depth_begin(), end = octree_.leaf_depth_end(); it != end; ++it) {
      size_t occupancyChangeCount = it.getLeafContainer().getCount();

      countMap.try_emplace(occupancyChangeCount, std::array<size_t, BUFFER_SIZE>{0, 0, 0});
      countMap.at(occupancyChangeCount).at(bufferIndex) = countMap.at(occupancyChangeCount).at(bufferIndex) + 1;
    }
  }

  std::ofstream ofs(filepath_);
  ofs << "Voxel Occupancy Change Count"
      << ","
      << "Voxel Count (Background)"
      << ","
      << "Voxel Count (Dynamic)"
      << ","
      << "Voxel Count (Other)" << endl;
  for (const auto& [occupancyChangeCount, countArray] : countMap) {
    ofs << occupancyChangeCount << "," << countArray.at(0) << "," << countArray.at(1) << "," << countArray.at(2)
        << endl;
  }
  octree_.deleteTree();
}

}  // namespace jpcc
