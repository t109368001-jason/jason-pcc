#include "VoxelPointCountToVoxelCount.h"

using namespace std;
using namespace std::filesystem;

namespace jpcc {

//////////////////////////////////////////////////////////////////////////////////////////////
VoxelPointCountToVoxelCount::VoxelPointCountToVoxelCount(const std::string& outputDir,
                                                         const std::string& filename,
                                                         const double       resolution) :
    Analyzer(outputDir, filename), octreeCounter_(resolution) {}

//////////////////////////////////////////////////////////////////////////////////////////////
VoxelPointCountToVoxelCount::VoxelPointCountToVoxelCount(const std::string& outputDir, const double resolution) :
    VoxelPointCountToVoxelCount(
        outputDir, "VoxelPointCountToVoxelCount[" + to_string(resolution) + "].csv", resolution) {}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelPointCountToVoxelCount::compute(FrameConstPtr background, FrameConstPtr dynamic, FrameConstPtr other) {
  octreeCounter_.addFrame(0, background);
  octreeCounter_.addFrame(1, dynamic);
  octreeCounter_.addFrame(2, other);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelPointCountToVoxelCount::finalCompute() {
  octree::OctreeCounter<PointNormal, 3>::CountMap countMap = octreeCounter_.getOccupancyCountToVoxelCount();
  std::ofstream                                   ofs(filepath_);
  ofs << "Voxel Point Count"
      << ","
      << "Voxel Count (Background)"
      << ","
      << "Voxel Count (Dynamic)"
      << ","
      << "Voxel Count (Other)" << endl;
  for (const auto& [occupancyCount, countArray] : countMap) {
    ofs << occupancyCount << "," << countArray.at(0) << "," << countArray.at(1) << "," << countArray.at(2) << endl;
  }
  octreeCounter_.deleteTree();
}

}  // namespace jpcc
