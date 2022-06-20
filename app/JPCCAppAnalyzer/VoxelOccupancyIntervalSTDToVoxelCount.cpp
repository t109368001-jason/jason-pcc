#include "VoxelOccupancyIntervalSTDToVoxelCount.h"

#include <cmath>
#include <map>

#include <Eigen/Dense>

using namespace std;
using namespace std::filesystem;
using namespace jpcc::octree;

namespace jpcc {

//////////////////////////////////////////////////////////////////////////////////////////////
VoxelOccupancyIntervalSTDToVoxelCount::VoxelOccupancyIntervalSTDToVoxelCount(const float&       frequency,
                                                                             const double&      resolution,
                                                                             const std::string& outputDir) :
    Analyzer(frequency, resolution, outputDir, "VoxelOccupancyIntervalSTDToVoxelCount"), octree_(resolution) {}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelOccupancyIntervalSTDToVoxelCount::compute(FrameConstPtr background,
                                                    FrameConstPtr dynamic,
                                                    FrameConstPtr other) {
  for (BufferIndex bufferIndex = 0; bufferIndex < BUFFER_SIZE; bufferIndex++) {
    octree_.switchBuffers(bufferIndex);
    for (auto it = octree_.leaf_depth_begin(), end = octree_.leaf_depth_end(); it != end; ++it) {
      OctreeContainerOccupancyInterval& leafNode = it.getLeafContainer();
      ++leafNode;
    }
  }
  octree_.addFrame(0, background);
  octree_.addFrame(1, dynamic);
  octree_.addFrame(2, other);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelOccupancyIntervalSTDToVoxelCount::finalCompute() {
  std::map<int, std::array<size_t, BUFFER_SIZE>> countMap;
  for (BufferIndex bufferIndex = 0; bufferIndex < BUFFER_SIZE; bufferIndex++) {
    octree_.switchBuffers(bufferIndex);
    for (auto it = octree_.leaf_depth_begin(), end = octree_.leaf_depth_end(); it != end; ++it) {
      const std::vector<int>& occupancyIntervals_ = it.getLeafContainer().getOccupancyIntervals();

      const Eigen::VectorXi occupancyIntervals =
          Eigen::Map<const Eigen::VectorXi, Eigen::Unaligned>(occupancyIntervals_.data(), occupancyIntervals_.size());

      double occupancyIntervalSTD = std::sqrt((occupancyIntervals.array() - occupancyIntervals.mean()).square().sum() /
                                              (double)occupancyIntervals.size());

      assert(!isnan(occupancyIntervalSTD));

      int quantizedOccupancyIntervalSTD = (int)std::round(occupancyIntervalSTD);

      countMap.try_emplace(quantizedOccupancyIntervalSTD, std::array<size_t, BUFFER_SIZE>{0, 0, 0});
      countMap.at(quantizedOccupancyIntervalSTD).at(bufferIndex) =
          countMap.at(quantizedOccupancyIntervalSTD).at(bufferIndex) + 1;
    }
  }

  std::ofstream ofs(filepath_);
  ofs << "Voxel Occupancy Interval STD"
      << ","
      << "Voxel Count (Background)"
      << ","
      << "Voxel Count (Dynamic)"
      << ","
      << "Voxel Count (Other)" << endl;
  for (const auto& [occupancyIntervalSTD, countArray] : countMap) {
    ofs << occupancyIntervalSTD << "," << countArray.at(0) << "," << countArray.at(1) << "," << countArray.at(2)
        << endl;
  }
  octree_.deleteTree();
}

}  // namespace jpcc
