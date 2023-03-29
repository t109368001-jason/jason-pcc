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
    Analyzer(frequency, resolution, outputDir, "VoxelOccupancyChangeCountToVoxelCount"), octree_(resolution) {
}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelOccupancyChangeCountToVoxelCount::compute(PclFrameConstPtr<PointAnalyzer> background,
                                                    PclFrameConstPtr<PointAnalyzer> dynamic,
                                                    PclFrameConstPtr<PointAnalyzer> other) {
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
      countMap[occupancyChangeCount][bufferIndex] = countMap[occupancyChangeCount][bufferIndex] + 1;
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
    ofs << occupancyChangeCount << "," << countArray[0] << "," << countArray[1] << "," << countArray[2] << endl;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelOccupancyChangeCountToVoxelCount::getCloud(PclFramePtr<PointAnalyzer>& cloud) {
  double min_x_, min_y_, min_z_, max_x_, max_y_, max_z_;
  octree_.getBoundingBox(min_x_, min_y_, min_z_, max_x_, max_y_, max_z_);

  cloud = jpcc::make_shared<PclFrame<PointAnalyzer>>();
  for (BufferIndex bufferIndex = 0; bufferIndex < BUFFER_SIZE; bufferIndex++) {
    octree_.switchBuffers(bufferIndex);
    for (auto it = octree_.leaf_depth_begin(), end = octree_.leaf_depth_end(); it != end; ++it) {
      auto x =
          static_cast<float>((static_cast<double>(it.getCurrentOctreeKey().x) + 0.5f) * this->resolution_ + min_x_);
      auto y =
          static_cast<float>((static_cast<double>(it.getCurrentOctreeKey().y) + 0.5f) * this->resolution_ + min_y_);
      auto z =
          static_cast<float>((static_cast<double>(it.getCurrentOctreeKey().z) + 0.5f) * this->resolution_ + min_z_);
      auto i = static_cast<float>(it.getLeafContainer().getCount());
      cloud->points.emplace_back(x, y, z, i);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelOccupancyChangeCountToVoxelCount::reset() {
  Analyzer::reset();
  octree_.deleteTree();
}

}  // namespace jpcc
