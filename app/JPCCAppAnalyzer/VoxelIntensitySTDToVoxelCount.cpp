#include "VoxelIntensitySTDToVoxelCount.h"

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
VoxelIntensitySTDToVoxelCount::VoxelIntensitySTDToVoxelCount(const float&       frequency,
                                                             const double&      resolution,
                                                             const std::string& outputDir) :
    Analyzer(frequency, resolution, outputDir, "VoxelIntensitySTDToVoxelCount"), octree_(resolution) {}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelIntensitySTDToVoxelCount::compute(FrameConstPtr background, FrameConstPtr dynamic, FrameConstPtr other) {
  octree_.addFrame(0, background);
  octree_.addFrame(1, dynamic);
  octree_.addFrame(2, other);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelIntensitySTDToVoxelCount::finalCompute() {
  map<int, array<size_t, BUFFER_SIZE>> countMap;
  for (BufferIndex bufferIndex = 0; bufferIndex < BUFFER_SIZE; bufferIndex++) {
    octree_.switchBuffers(bufferIndex);
    for (auto it = octree_.leaf_depth_begin(), end = octree_.leaf_depth_end(); it != end; ++it) {
      const vector<float>& intensities = it.getLeafContainer().getIntensities();
      if (intensities.empty()) { continue; }

      double intensitySTD = standard_deviation(intensities);

      assert(!isnan(intensitySTD));

      auto quantizedIntensitySTD = (int)round(intensitySTD);

      countMap.try_emplace(quantizedIntensitySTD, array<size_t, BUFFER_SIZE>{0, 0, 0});
      countMap.at(quantizedIntensitySTD).at(bufferIndex) = countMap.at(quantizedIntensitySTD).at(bufferIndex) + 1;
    }
  }

  ofstream ofs(filepath_);
  ofs << "Voxel Intensity STD"
      << ","
      << "Voxel Count (Background)"
      << ","
      << "Voxel Count (Dynamic)"
      << ","
      << "Voxel Count (Other)" << endl;
  for (const auto& [quantizedIntensitySTD, countArray] : countMap) {
    ofs << to_string(quantizedIntensitySTD) << "," << countArray.at(0) << "," << countArray.at(1) << ","
        << countArray.at(2) << endl;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelIntensitySTDToVoxelCount::getCloud(FramePtr cloud) {
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

      const vector<float>& intensities = it.getLeafContainer().getIntensities();
      if (intensities.empty()) { continue; }

      double intensitySTD = standard_deviation(intensities);

      assert(!isnan(intensitySTD));

      auto quantizedIntensitySTD = (int)round(intensitySTD);

      auto i = static_cast<float>(quantizedIntensitySTD);
      cloud->points.emplace_back(x, y, z, i);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelIntensitySTDToVoxelCount::reset() {
  Analyzer::reset();
  octree_.deleteTree();
}

}  // namespace jpcc
