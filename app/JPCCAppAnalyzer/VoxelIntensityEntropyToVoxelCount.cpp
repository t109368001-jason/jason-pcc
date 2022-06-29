#include "VoxelIntensityEntropyToVoxelCount.h"

#include <cmath>
#include <map>

#include <Eigen/Dense>

#include <jpcc/math/Math.h>

using namespace std;
using namespace std::filesystem;
using namespace jpcc::octree;

namespace jpcc {

//////////////////////////////////////////////////////////////////////////////////////////////
VoxelIntensityEntropyToVoxelCount::VoxelIntensityEntropyToVoxelCount(const float&       frequency,
                                                                     const double&      resolution,
                                                                     const std::string& outputDir) :
    Analyzer(frequency, resolution, outputDir, "VoxelIntensityEntropyToVoxelCount"), octree_(resolution) {}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelIntensityEntropyToVoxelCount::compute(FrameConstPtr background, FrameConstPtr dynamic, FrameConstPtr other) {
  octree_.addFrame(0, background);
  octree_.addFrame(1, dynamic);
  octree_.addFrame(2, other);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelIntensityEntropyToVoxelCount::finalCompute() {
  map<int, array<size_t, BUFFER_SIZE>> countMap;
  for (BufferIndex bufferIndex = 0; bufferIndex < BUFFER_SIZE; bufferIndex++) {
    octree_.switchBuffers(bufferIndex);
    for (auto it = octree_.leaf_depth_begin(), end = octree_.leaf_depth_end(); it != end; ++it) {
      const vector<float>& intensities = it.getLeafContainer().getIntensities();
      if (intensities.empty()) { continue; }

      double intensityEntropy = entropy(intensities, 0.0f, 255.0f, 10.0f);

      auto quantizedIntensityEntropy = (int)round(intensityEntropy * 10.0);

      countMap.try_emplace(quantizedIntensityEntropy, array<size_t, BUFFER_SIZE>{0, 0, 0});
      countMap.at(quantizedIntensityEntropy).at(bufferIndex) =
          countMap.at(quantizedIntensityEntropy).at(bufferIndex) + 1;
    }
  }

  ofstream ofs(filepath_);
  ofs << "Voxel Intensity Entropy"
      << ","
      << "Voxel Count (Background)"
      << ","
      << "Voxel Count (Dynamic)"
      << ","
      << "Voxel Count (Other)" << endl;
  for (const auto& [quantizedIntensityEntropy, countArray] : countMap) {
    ofs << to_string(quantizedIntensityEntropy) << "," << countArray.at(0) << "," << countArray.at(1) << ","
        << countArray.at(2) << endl;
  }
  octree_.deleteTree();
}

}  // namespace jpcc
