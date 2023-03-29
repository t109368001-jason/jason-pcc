#include "VoxelOccupancyIntervalSTDToVoxelCount.h"

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
VoxelOccupancyIntervalSTDToVoxelCount::VoxelOccupancyIntervalSTDToVoxelCount(const float&  frequency,
                                                                             const double& resolution,
                                                                             const string& outputDir) :
    Analyzer(frequency, resolution, outputDir, "VoxelOccupancyIntervalSTDToVoxelCount"), octree_(resolution) {
}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelOccupancyIntervalSTDToVoxelCount::compute(PclFrameConstPtr<PointAnalyzer> background,
                                                    PclFrameConstPtr<PointAnalyzer> dynamic,
                                                    PclFrameConstPtr<PointAnalyzer> other) {
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
  map<int, array<size_t, BUFFER_SIZE>> countMap;
  for (BufferIndex bufferIndex = 0; bufferIndex < BUFFER_SIZE; bufferIndex++) {
    octree_.switchBuffers(bufferIndex);
    for (auto it = octree_.leaf_depth_begin(), end = octree_.leaf_depth_end(); it != end; ++it) {
      const vector<int>& occupancyIntervals = it.getLeafContainer().getOccupancyIntervals();
      if (occupancyIntervals.empty()) {
        continue;
      }

      double occupancyIntervalSTD = standard_deviation(occupancyIntervals);

      assert(!isnan(occupancyIntervalSTD));

      int quantizedOccupancyIntervalSTD = static_cast<int>(round(occupancyIntervalSTD));

      countMap.try_emplace(quantizedOccupancyIntervalSTD, array<size_t, BUFFER_SIZE>{0, 0, 0});
      countMap[quantizedOccupancyIntervalSTD][bufferIndex] = countMap[quantizedOccupancyIntervalSTD][bufferIndex] + 1;
    }
  }

  ofstream ofs(filepath_);
  ofs << "Voxel Occupancy Interval STD"
      << ","
      << "Voxel Count (Background)"
      << ","
      << "Voxel Count (Dynamic)"
      << ","
      << "Voxel Count (Other)" << endl;
  for (const auto& [occupancyIntervalSTD, countArray] : countMap) {
    ofs << occupancyIntervalSTD << "," << countArray[0] << "," << countArray[1] << "," << countArray[2] << endl;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelOccupancyIntervalSTDToVoxelCount::getCloud(PclFramePtr<PointAnalyzer>& cloud) {
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

      const vector<int>& occupancyIntervals = it.getLeafContainer().getOccupancyIntervals();
      if (occupancyIntervals.empty()) {
        continue;
      }

      double occupancyIntervalSTD = standard_deviation(occupancyIntervals);

      assert(!isnan(occupancyIntervalSTD));

      int quantizedOccupancyIntervalSTD = static_cast<int>(round(occupancyIntervalSTD));

      auto i = static_cast<float>(quantizedOccupancyIntervalSTD);
      cloud->points.emplace_back(x, y, z, i);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelOccupancyIntervalSTDToVoxelCount::reset() {
  Analyzer::reset();
  octree_.deleteTree();
}

}  // namespace jpcc
