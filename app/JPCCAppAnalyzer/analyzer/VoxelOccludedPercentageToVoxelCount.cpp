#include "VoxelOccludedPercentageToVoxelCount.h"

#include <execution>
#include <map>

using namespace std;
using namespace std::filesystem;
using namespace Eigen;
using namespace jpcc::octree;

namespace jpcc {

//////////////////////////////////////////////////////////////////////////////////////////////
VoxelOccludedPercentageToVoxelCount::VoxelOccludedPercentageToVoxelCount(const float&  frequency,
                                                                         const double& resolution,
                                                                         const string& outputDir,
                                                                         const size_t  quantResolution) :
    Analyzer(frequency,
             resolution,
             outputDir,
             "VoxelOccludedPercentageToVoxelCount",
             "[" + to_string(quantResolution) + "]"),
    quantResolution_(quantResolution),
    quantCount_((size_t)(resolution / (double)quantResolution)),
    octree_(resolution) {}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelOccludedPercentageToVoxelCount::compute(FrameConstPtr background,
                                                  FrameConstPtr dynamic,
                                                  FrameConstPtr other) {
  octree_.addFrame(0, background);
  octree_.addFrame(1, dynamic);
  octree_.addFrame(2, other);
  for (BufferIndex bufferIndex = 0; bufferIndex < BUFFER_SIZE; bufferIndex++) {
    octree_.switchBuffers(bufferIndex);
    for (auto it = octree_.leaf_depth_begin(), end = octree_.leaf_depth_end(); it != end; ++it) {
      OctreeContainerOccludedCount& leafNode = it.getLeafContainer();
      Eigen::Vector3f               min_pt;
      Eigen::Vector3f               max_pt;
      octree_.getVoxelBounds(it, min_pt, max_pt);
      leafNode.compute(min_pt, max_pt, quantCount_);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelOccludedPercentageToVoxelCount::finalCompute() {
  map<int, array<size_t, BUFFER_SIZE>> countMap;
  for (BufferIndex bufferIndex = 0; bufferIndex < BUFFER_SIZE; bufferIndex++) {
    octree_.switchBuffers(bufferIndex);
    for (auto it = octree_.leaf_depth_begin(), end = octree_.leaf_depth_end(); it != end; ++it) {
      OctreeContainerOccludedCount& leafNode = it.getLeafContainer();

      int percentage = (int)leafNode.getMinimumOccludedPercentage(quantCount_);

      countMap.try_emplace(percentage, array<size_t, BUFFER_SIZE>{0, 0, 0});
      countMap.at(percentage).at(bufferIndex)++;
    }
  }

  ofstream ofs(filepath_);
  ofs << "Voxel Occluded Percentage"
      << ","
      << "Voxel Count (Background)"
      << ","
      << "Voxel Count (Dynamic)"
      << ","
      << "Voxel Count (Other)" << endl;
  for (const auto& [percentage, countArray] : countMap) {
    ofs << percentage << "," << countArray.at(0) << "," << countArray.at(1) << "," << countArray.at(2) << endl;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelOccludedPercentageToVoxelCount::getCloud(FramePtr& cloud) {
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

      OctreeContainerOccludedCount& leafNode = it.getLeafContainer();

      int percentage = (int)leafNode.getMinimumOccludedPercentage(quantCount_);

      auto i = static_cast<float>(percentage);
      cloud->points.emplace_back(x, y, z, i);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelOccludedPercentageToVoxelCount::reset() {
  Analyzer::reset();
  octree_.deleteTree();
}

}  // namespace jpcc
