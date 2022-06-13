#include "VoxelPointNormalAngleSTDToVoxelCount.h"

#include <cmath>
#include <map>

using namespace std;
using namespace std::filesystem;
using namespace jpcc::octree;

namespace jpcc {

//////////////////////////////////////////////////////////////////////////////////////////////
VoxelPointNormalAngleSTDToVoxelCount::VoxelPointNormalAngleSTDToVoxelCount(const std::string& outputDir,
                                                                           const double       resolution) :
    Analyzer(outputDir, "VoxelPointNormalAngleSTDToVoxelCount[" + to_string(resolution) + "].csv"),
    octree_(resolution) {}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelPointNormalAngleSTDToVoxelCount::compute(FrameConstPtr background,
                                                   FrameConstPtr dynamic,
                                                   FrameConstPtr other) {
  octree_.addFrame(0, background);
  octree_.addFrame(1, dynamic);
  octree_.addFrame(2, other);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelPointNormalAngleSTDToVoxelCount::finalCompute() {
  std::map<int, std::array<size_t, BUFFER_SIZE * 2>> countMap;
  for (BufferIndex bufferIndex = 0; bufferIndex < BUFFER_SIZE; bufferIndex++) {
    octree_.switchBuffers(bufferIndex);
    for (auto it = octree_.leaf_depth_begin(), end = octree_.leaf_depth_end(); it != end; ++it) {
      const Eigen::VectorXd azimuths = it.getLeafContainer().getAzimuths();
      const Eigen::VectorXd zeniths  = it.getLeafContainer().getZeniths();

      double azimuthSTD = std::sqrt((azimuths.array() - azimuths.mean()).square().sum() / (double)azimuths.size()) *
                          TO_DEGREE_MULTIPLIER;
      double zenithSTD =
          std::sqrt((zeniths.array() - zeniths.mean()).square().sum() / (double)zeniths.size()) * TO_DEGREE_MULTIPLIER;

      assert(!isnan(azimuthSTD));
      assert(!isnan(zenithSTD));

      int quantizedAzimuthSTD = (int)std::round(azimuthSTD);
      int quantizedZenithSTD  = (int)std::round(zenithSTD);

      countMap.try_emplace(quantizedAzimuthSTD, std::array<size_t, BUFFER_SIZE * 2>{0, 0, 0, 0, 0, 0});
      countMap.try_emplace(quantizedZenithSTD, std::array<size_t, BUFFER_SIZE * 2>{0, 0, 0, 0, 0, 0});
      countMap.at(quantizedAzimuthSTD).at(bufferIndex) = countMap.at(quantizedAzimuthSTD).at(bufferIndex) + 1;
      countMap.at(quantizedZenithSTD).at(bufferIndex + BUFFER_SIZE) =
          countMap.at(quantizedZenithSTD).at(bufferIndex + BUFFER_SIZE) + 1;
    }
  }

  std::ofstream ofs(filepath_);
  ofs << ""
      << ","
      << "azimuth"
      << ","
      << "azimuth"
      << ","
      << "azimuth"
      << ","
      << "zenith"
      << ","
      << "zenith"
      << ","
      << "zenith" << endl;
  ofs << "Voxel Point Normal Angle STD"
      << ","
      << "Voxel Count (Background)"
      << ","
      << "Voxel Count (Dynamic)"
      << ","
      << "Voxel Count (Other)"
      << ","
      << "Voxel Count (Background)"
      << ","
      << "Voxel Count (Dynamic)"
      << ","
      << "Voxel Count (Other)" << endl;
  for (const auto& [angleSTD, countArray] : countMap) {
    ofs << angleSTD << "," << countArray.at(0) << "," << countArray.at(1) << "," << countArray.at(2) << ","
        << countArray.at(3) << "," << countArray.at(4) << "," << countArray.at(5) << endl;
  }
  octree_.deleteTree();
}

}  // namespace jpcc
