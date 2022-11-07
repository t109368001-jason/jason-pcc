#include "VoxelIntensityEntropyToVoxelCount.h"

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
VoxelIntensityEntropyToVoxelCount::VoxelIntensityEntropyToVoxelCount(const float&       frequency,
                                                                     const double&      resolution,
                                                                     const std::string& outputDir) :
    Analyzer(frequency, resolution, outputDir, "VoxelIntensityEntropyToVoxelCount"), octree_(resolution) {}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelIntensityEntropyToVoxelCount::compute(PclFrameConstPtr<PointAnalyzer> background,
                                                PclFrameConstPtr<PointAnalyzer> dynamic,
                                                PclFrameConstPtr<PointAnalyzer> other) {
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

      double intensityEntropy = entropy(intensities, 0.0f, 1.0f, 0.04f);

      auto quantizedIntensityEntropy = (int)round(intensityEntropy * 10.0);

      countMap.try_emplace(quantizedIntensityEntropy, array<size_t, BUFFER_SIZE>{0, 0, 0});
      countMap[quantizedIntensityEntropy][bufferIndex] = countMap[quantizedIntensityEntropy][bufferIndex] + 1;
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
    ofs << to_string(quantizedIntensityEntropy) << "," << countArray[0] << "," << countArray[1] << "," << countArray[2]
        << endl;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelIntensityEntropyToVoxelCount::getCloud(PclFramePtr<PointAnalyzer>& cloud) {
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

      const vector<float>& intensities = it.getLeafContainer().getIntensities();
      if (intensities.empty()) { continue; }

      double intensityEntropy = entropy(intensities, 0.0f, 1.0f, 0.04f);

      auto quantizedIntensityEntropy = (int)round(intensityEntropy * 10.0);

      auto i = static_cast<float>(quantizedIntensityEntropy);
      cloud->points.emplace_back(x, y, z, i);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelIntensityEntropyToVoxelCount::reset() {
  Analyzer::reset();
  octree_.deleteTree();
}

}  // namespace jpcc
