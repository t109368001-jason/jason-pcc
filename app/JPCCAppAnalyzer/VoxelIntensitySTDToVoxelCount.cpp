#include "VoxelIntensitySTDToVoxelCount.h"

#include <cmath>
#include <map>

#include <Eigen/Dense>

#include <jpcc/math/Math.h>

using namespace std;
using namespace std::filesystem;
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
  octree_.deleteTree();
}

}  // namespace jpcc
