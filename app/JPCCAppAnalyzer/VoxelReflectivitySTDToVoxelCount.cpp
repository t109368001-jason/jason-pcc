#include "VoxelReflectivitySTDToVoxelCount.h"

#include <cmath>
#include <map>

#include <Eigen/Dense>

using namespace std;
using namespace std::filesystem;
using namespace jpcc::octree;

namespace jpcc {

//////////////////////////////////////////////////////////////////////////////////////////////
VoxelReflectivitySTDToVoxelCount::VoxelReflectivitySTDToVoxelCount(const float&       frequency,
                                                                   const double&      resolution,
                                                                   const std::string& outputDir) :
    Analyzer(frequency, resolution, outputDir, "VoxelReflectivitySTDToVoxelCount"), octree_(resolution) {}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelReflectivitySTDToVoxelCount::compute(FrameConstPtr background, FrameConstPtr dynamic, FrameConstPtr other) {
  octree_.addFrame(0, background);
  octree_.addFrame(1, dynamic);
  octree_.addFrame(2, other);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelReflectivitySTDToVoxelCount::finalCompute() {
  std::map<Reflectivity, std::array<size_t, BUFFER_SIZE>> countMap;
  for (BufferIndex bufferIndex = 0; bufferIndex < BUFFER_SIZE; bufferIndex++) {
    octree_.switchBuffers(bufferIndex);
    for (auto it = octree_.leaf_depth_begin(), end = octree_.leaf_depth_end(); it != end; ++it) {
      const std::vector<Reflectivity>& reflectivitys_ = it.getLeafContainer().getReflectivitys();
      if (reflectivitys_.empty()) { continue; }
      vector<float> reflectivitysFloat(reflectivitys_.size());
      transform(reflectivitys_.begin(), reflectivitys_.end(), reflectivitysFloat.begin(),
                [](const auto& o) { return o; });

      const Eigen::VectorXf reflectivitys =
          Eigen::Map<const Eigen::VectorXf, Eigen::Unaligned>(reflectivitysFloat.data(), reflectivitysFloat.size());

      double reflectivitySTD =
          std::sqrt((reflectivitys.array() - reflectivitys.mean()).square().sum() / (double)reflectivitys.size());

      assert(!isnan(reflectivitySTD));

      auto quantizedReflectivitySTD = (Reflectivity)std::round(reflectivitySTD);

      countMap.try_emplace(quantizedReflectivitySTD, std::array<size_t, BUFFER_SIZE>{0, 0, 0});
      countMap.at(quantizedReflectivitySTD).at(bufferIndex) = countMap.at(quantizedReflectivitySTD).at(bufferIndex) + 1;
    }
  }

  std::ofstream ofs(filepath_);
  ofs << "Voxel Reflectivity STD"
      << ","
      << "Voxel Count (Background)"
      << ","
      << "Voxel Count (Dynamic)"
      << ","
      << "Voxel Count (Other)" << endl;
  for (const auto& [quantizedReflectivitySTD, countArray] : countMap) {
    ofs << to_string(quantizedReflectivitySTD) << "," << countArray.at(0) << "," << countArray.at(1) << ","
        << countArray.at(2) << endl;
  }
  octree_.deleteTree();
}

}  // namespace jpcc
