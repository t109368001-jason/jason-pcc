#include "VoxelPointNormalAngleSTDToVoxelCount.h"

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
VoxelPointNormalAngleSTDToVoxelCount::VoxelPointNormalAngleSTDToVoxelCount(const float&  frequency,
                                                                           const double& resolution,
                                                                           const string& outputDir) :
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
  map<int, array<size_t, BUFFER_SIZE * 2>> countMap;
  for (BufferIndex bufferIndex = 0; bufferIndex < BUFFER_SIZE; bufferIndex++) {
    octree_.switchBuffers(bufferIndex);
    for (auto it = octree_.leaf_depth_begin(), end = octree_.leaf_depth_end(); it != end; ++it) {
      const vector<double>& azimuths_ = it.getLeafContainer().getAzimuths();
      const vector<double>& zeniths_  = it.getLeafContainer().getZeniths();
      if (azimuths_.empty() && zeniths_.empty()) { continue; }

      double azimuthSTD = standard_deviation(azimuths_);
      double zenithSTD  = standard_deviation(zeniths_);

      azimuthSTD *= TO_DEGREE_MULTIPLIER;
      zenithSTD *= TO_DEGREE_MULTIPLIER;

      assert(!isnan(azimuthSTD));
      assert(!isnan(zenithSTD));

      int quantizedAzimuthSTD = (int)round(azimuthSTD);
      int quantizedZenithSTD  = (int)round(zenithSTD);

      countMap.try_emplace(quantizedAzimuthSTD, array<size_t, BUFFER_SIZE * 2>{0, 0, 0, 0, 0, 0});
      countMap.try_emplace(quantizedZenithSTD, array<size_t, BUFFER_SIZE * 2>{0, 0, 0, 0, 0, 0});
      countMap.at(quantizedAzimuthSTD).at(bufferIndex) = countMap.at(quantizedAzimuthSTD).at(bufferIndex) + 1;
      countMap.at(quantizedZenithSTD).at(bufferIndex + BUFFER_SIZE) =
          countMap.at(quantizedZenithSTD).at(bufferIndex + BUFFER_SIZE) + 1;
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
}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelPointNormalAngleSTDToVoxelCount::getCloud(FramePtr cloud) {
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

      const vector<double>& azimuths_ = it.getLeafContainer().getAzimuths();
      if (azimuths_.empty()) { continue; }

      double azimuthSTD = standard_deviation(azimuths_);

      azimuthSTD *= TO_DEGREE_MULTIPLIER;

      assert(!isnan(azimuthSTD));

      int quantizedAzimuthSTD = (int)round(azimuthSTD);

      auto i = static_cast<float>(quantizedAzimuthSTD);
      cloud->points.emplace_back(x, y, z, i);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelPointNormalAngleSTDToVoxelCount::reset() {
  Analyzer::reset();
  octree_.deleteTree();
}

}  // namespace jpcc
