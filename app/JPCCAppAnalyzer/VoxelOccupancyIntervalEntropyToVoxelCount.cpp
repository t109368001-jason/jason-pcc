#include "VoxelOccupancyIntervalEntropyToVoxelCount.h"

#include <cmath>
#include <map>

#include <Eigen/Dense>

#include <jpcc/math/Math.h>

using namespace std;
using namespace std::filesystem;
using namespace jpcc::math;
using namespace jpcc::octree;

namespace jpcc {

//////////////////////////////////////////////////////////////////////////////////////////////
VoxelOccupancyIntervalEntropyToVoxelCount::VoxelOccupancyIntervalEntropyToVoxelCount(const float&  frequency,
                                                                                     const double& resolution,
                                                                                     const string& outputDir) :
    Analyzer(frequency, resolution, outputDir, "VoxelOccupancyIntervalEntropyToVoxelCount"), octree_(resolution) {}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelOccupancyIntervalEntropyToVoxelCount::compute(FrameConstPtr background,
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
void VoxelOccupancyIntervalEntropyToVoxelCount::finalCompute() {
  map<int, array<size_t, BUFFER_SIZE>> countMap;
  for (BufferIndex bufferIndex = 0; bufferIndex < BUFFER_SIZE; bufferIndex++) {
    octree_.switchBuffers(bufferIndex);
    for (auto it = octree_.leaf_depth_begin(), end = octree_.leaf_depth_end(); it != end; ++it) {
      const vector<int>& occupancyIntervals = it.getLeafContainer().getOccupancyIntervals();
      if (occupancyIntervals.empty()) { continue; }

      double occupancyIntervalEntropy = entropy(occupancyIntervals, 0, 6372, 10);

      int quantizedOccupancyIntervalEntropy = (int)round(occupancyIntervalEntropy * 10.0);

      countMap.try_emplace(quantizedOccupancyIntervalEntropy, array<size_t, BUFFER_SIZE>{0, 0, 0});
      countMap.at(quantizedOccupancyIntervalEntropy).at(bufferIndex) =
          countMap.at(quantizedOccupancyIntervalEntropy).at(bufferIndex) + 1;
    }
  }

  ofstream ofs(filepath_);
  ofs << "Voxel Occupancy Interval Entropy"
      << ","
      << "Voxel Count (Background)"
      << ","
      << "Voxel Count (Dynamic)"
      << ","
      << "Voxel Count (Other)" << endl;
  for (const auto& [occupancyIntervalEntropy, countArray] : countMap) {
    ofs << occupancyIntervalEntropy << "," << countArray.at(0) << "," << countArray.at(1) << "," << countArray.at(2)
        << endl;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelOccupancyIntervalEntropyToVoxelCount::getCloud(FramePtr& cloud) {
  double min_x_, min_y_, min_z_, max_x_, max_y_, max_z_;
  octree_.getBoundingBox(min_x_, min_y_, min_z_, max_x_, max_y_, max_z_);

  cloud = jpcc::make_shared<Frame>();
  for (BufferIndex bufferIndex = 0; bufferIndex < BUFFER_SIZE; bufferIndex++) {
    octree_.switchBuffers(bufferIndex);
    for (auto it = octree_.leaf_depth_begin(), end = octree_.leaf_depth_end(); it != end; ++it) {
      auto x =
          static_cast<float>((static_cast<double>(it.getCurrentOctreeKey().x) + 0.5f) * this->resolution_ + min_x_);
      auto y =
          static_cast<float>((static_cast<double>(it.getCurrentOctreeKey().y) + 0.5f) * this->resolution_ + min_y_);
      auto z =
          static_cast<float>((static_cast<double>(it.getCurrentOctreeKey().z) + 0.5f) * this->resolution_ + min_z_);

      const vector<int>& occupancyIntervals = it.getLeafContainer().getOccupancyIntervals();
      if (occupancyIntervals.empty()) { continue; }

      double occupancyIntervalEntropy = entropy(occupancyIntervals, 0, 6372, 10);

      int quantizedOccupancyIntervalEntropy = (int)round(occupancyIntervalEntropy * 10.0);

      auto i = static_cast<float>(quantizedOccupancyIntervalEntropy);
      cloud->points.emplace_back(x, y, z, i);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelOccupancyIntervalEntropyToVoxelCount::reset() {
  Analyzer::reset();
  octree_.deleteTree();
}

}  // namespace jpcc
