#include "VoxelOccludedPercentageToVoxelCount.h"

#include <execution>
#include <map>

using namespace std;
using namespace std::filesystem;
using namespace Eigen;
using namespace jpcc::octree;

namespace jpcc {

//////////////////////////////////////////////////////////////////////////////////////////////
VoxelOccludedPercentageToVoxelCount::VoxelOccludedPercentageToVoxelCount(const float&       frequency,
                                                                         const double&      resolution,
                                                                         const std::string& outputDir,
                                                                         const size_t       quantResolution) :
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

      countMap.try_emplace(percentage, std::array<size_t, BUFFER_SIZE>{0, 0, 0});
      countMap.at(percentage).at(bufferIndex)++;
    }
  }

  std::ofstream ofs(filepath_);
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
  octree_.deleteTree();
}

}  // namespace jpcc