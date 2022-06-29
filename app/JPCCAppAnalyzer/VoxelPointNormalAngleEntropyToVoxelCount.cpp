#include "VoxelPointNormalAngleEntropyToVoxelCount.h"

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
VoxelPointNormalAngleEntropyToVoxelCount::VoxelPointNormalAngleEntropyToVoxelCount(const float&  frequency,
                                                                                   const double& resolution,
                                                                                   const string& outputDir) :
    Analyzer(frequency, resolution, outputDir, "VoxelPointNormalAngleEntropyToVoxelCount"), octree_(resolution) {}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelPointNormalAngleEntropyToVoxelCount::compute(FrameConstPtr background,
                                                       FrameConstPtr dynamic,
                                                       FrameConstPtr other) {
  octree_.addFrame(0, background);
  octree_.addFrame(1, dynamic);
  octree_.addFrame(2, other);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelPointNormalAngleEntropyToVoxelCount::finalCompute() {
  map<int, array<size_t, BUFFER_SIZE * 2>> countMap;
  for (BufferIndex bufferIndex = 0; bufferIndex < BUFFER_SIZE; bufferIndex++) {
    octree_.switchBuffers(bufferIndex);
    for (auto it = octree_.leaf_depth_begin(), end = octree_.leaf_depth_end(); it != end; ++it) {
      const vector<double>& azimuths_ = it.getLeafContainer().getAzimuths();
      const vector<double>& zeniths_  = it.getLeafContainer().getZeniths();
      if (azimuths_.empty() && zeniths_.empty()) { continue; }

      double azimuthEntropy = entropy(azimuths_, 0.0, M_PI * 2, M_PI * 2 / 10.0);
      double zenithEntropy  = entropy(zeniths_, 0.0, M_PI, M_PI / 10.0);

      int quantizedAzimuthEntropy = (int)round(azimuthEntropy * 10.0);
      int quantizedZenithEntropy  = (int)round(zenithEntropy * 10.0);

      countMap.try_emplace(quantizedAzimuthEntropy, array<size_t, BUFFER_SIZE * 2>{0, 0, 0, 0, 0, 0});
      countMap.try_emplace(quantizedZenithEntropy, array<size_t, BUFFER_SIZE * 2>{0, 0, 0, 0, 0, 0});
      countMap.at(quantizedAzimuthEntropy).at(bufferIndex) = countMap.at(quantizedAzimuthEntropy).at(bufferIndex) + 1;
      countMap.at(quantizedZenithEntropy).at(bufferIndex + BUFFER_SIZE) =
          countMap.at(quantizedZenithEntropy).at(bufferIndex + BUFFER_SIZE) + 1;
    }
  }

  ofstream ofs(filepath_);
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
  ofs << "Voxel Point Normal Angle Entropy"
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
  for (const auto& [angleEntropy, countArray] : countMap) {
    ofs << angleEntropy << "," << countArray.at(0) << "," << countArray.at(1) << "," << countArray.at(2) << ","
        << countArray.at(3) << "," << countArray.at(4) << "," << countArray.at(5) << endl;
  }
  octree_.deleteTree();
}

}  // namespace jpcc
