#include "VoxelPointNormalAngleSTDToVoxelCount.h"

#include <cmath>
#include <map>

#include <Eigen/Dense>

using namespace std;
using namespace std::filesystem;
using namespace jpcc::octree;

namespace jpcc {

//////////////////////////////////////////////////////////////////////////////////////////////
VoxelPointNormalAngleSTDToVoxelCount::VoxelPointNormalAngleSTDToVoxelCount(const float&       frequency,
                                                                           const double&      resolution,
                                                                           const std::string& outputDir) :
    Analyzer(frequency, resolution, outputDir, "VoxelPointNormalAngleSTDToVoxelCount"), octree_(resolution) {}

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
      const std::vector<double>& azimuths_ = it.getLeafContainer().getAzimuths();
      const std::vector<double>& zeniths_  = it.getLeafContainer().getZeniths();

      const Eigen::VectorXd azimuths =
          Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(azimuths_.data(), azimuths_.size());
      const Eigen::VectorXd zeniths =
          Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(zeniths_.data(), zeniths_.size());

      double azimuthSTD = std::sqrt((azimuths.array() - azimuths.mean()).square().sum() / (double)azimuths.size());
      double zenithSTD  = std::sqrt((zeniths.array() - zeniths.mean()).square().sum() / (double)zeniths.size());

      azimuthSTD *= TO_DEGREE_MULTIPLIER;
      zenithSTD *= TO_DEGREE_MULTIPLIER;

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
