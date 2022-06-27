#include "VoxelIntensitySTDToVoxelCount.h"

#include <cmath>
#include <map>

#include <Eigen/Dense>

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
  std::map<int, std::array<size_t, BUFFER_SIZE>> countMap;
  for (BufferIndex bufferIndex = 0; bufferIndex < BUFFER_SIZE; bufferIndex++) {
    octree_.switchBuffers(bufferIndex);
    for (auto it = octree_.leaf_depth_begin(), end = octree_.leaf_depth_end(); it != end; ++it) {
      const std::vector<float>& intensities_ = it.getLeafContainer().getIntensities();
      if (intensities_.empty()) { continue; }

      const Eigen::VectorXf intensities =
          Eigen::Map<const Eigen::VectorXf, Eigen::Unaligned>(intensities_.data(), intensities_.size());

      double intensitySTD =
          std::sqrt((intensities.array() - intensities.mean()).square().sum() / (double)intensities.size());

      assert(!isnan(intensitySTD));

      auto quantizedIntensitySTD = (int)std::round(intensitySTD);

      countMap.try_emplace(quantizedIntensitySTD, std::array<size_t, BUFFER_SIZE>{0, 0, 0});
      countMap.at(quantizedIntensitySTD).at(bufferIndex) = countMap.at(quantizedIntensitySTD).at(bufferIndex) + 1;
    }
  }

  std::ofstream ofs(filepath_);
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
