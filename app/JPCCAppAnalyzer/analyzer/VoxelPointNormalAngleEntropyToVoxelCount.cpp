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
void VoxelPointNormalAngleEntropyToVoxelCount::compute(FrameConstPtr<pcl::PointXYZINormal> background,
                                                       FrameConstPtr<pcl::PointXYZINormal> dynamic,
                                                       FrameConstPtr<pcl::PointXYZINormal> other) {
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
      const std::vector<double>& azimuths_ = it.getLeafContainer().getAzimuths();
      const std::vector<double>& zeniths_  = it.getLeafContainer().getZeniths();
      if (azimuths_.empty() && zeniths_.empty()) { continue; }

      double azimuthEntropy = entropy(azimuths_, 0.0, M_PI * 2, M_PI * 2 / 10.0);
      double zenithEntropy  = entropy(zeniths_, 0.0, M_PI, M_PI / 10.0);

      int quantizedAzimuthEntropy = (int)round(azimuthEntropy * 10.0);
      int quantizedZenithEntropy  = (int)round(zenithEntropy * 10.0);

      countMap.try_emplace(quantizedAzimuthEntropy, array<size_t, BUFFER_SIZE * 2>{0, 0, 0, 0, 0, 0});
      countMap.try_emplace(quantizedZenithEntropy, array<size_t, BUFFER_SIZE * 2>{0, 0, 0, 0, 0, 0});
      countMap[quantizedAzimuthEntropy][bufferIndex] = countMap[quantizedAzimuthEntropy][bufferIndex] + 1;
      countMap[quantizedZenithEntropy][bufferIndex + BUFFER_SIZE] =
          countMap[quantizedZenithEntropy][bufferIndex + BUFFER_SIZE] + 1;
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
    ofs << angleEntropy << "," << countArray[0] << "," << countArray[1] << "," << countArray[2] << "," << countArray[3]
        << "," << countArray[4] << "," << countArray[5] << endl;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelPointNormalAngleEntropyToVoxelCount::getCloud(FramePtr<pcl::PointXYZINormal>& cloud) {
  double min_x_, min_y_, min_z_, max_x_, max_y_, max_z_;
  octree_.getBoundingBox(min_x_, min_y_, min_z_, max_x_, max_y_, max_z_);

  cloud = jpcc::make_shared<Frame<pcl::PointXYZINormal>>();
  for (BufferIndex bufferIndex = 0; bufferIndex < BUFFER_SIZE; bufferIndex++) {
    octree_.switchBuffers(bufferIndex);
    for (auto it = octree_.leaf_depth_begin(), end = octree_.leaf_depth_end(); it != end; ++it) {
      auto x =
          static_cast<float>((static_cast<double>(it.getCurrentOctreeKey().x) + 0.5f) * this->resolution_ + min_x_);
      auto y =
          static_cast<float>((static_cast<double>(it.getCurrentOctreeKey().y) + 0.5f) * this->resolution_ + min_y_);
      auto z =
          static_cast<float>((static_cast<double>(it.getCurrentOctreeKey().z) + 0.5f) * this->resolution_ + min_z_);

      const std::vector<double>& azimuths_ = it.getLeafContainer().getAzimuths();
      if (azimuths_.empty()) { continue; }

      double azimuthEntropy = entropy(azimuths_, 0.0, M_PI * 2, M_PI * 2 / 10.0);

      int quantizedAzimuthEntropy = (int)round(azimuthEntropy * 10.0);

      auto i = static_cast<float>(quantizedAzimuthEntropy);
      cloud->points.emplace_back(x, y, z, i);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelPointNormalAngleEntropyToVoxelCount::reset() {
  Analyzer::reset();
  octree_.deleteTree();
}

}  // namespace jpcc
