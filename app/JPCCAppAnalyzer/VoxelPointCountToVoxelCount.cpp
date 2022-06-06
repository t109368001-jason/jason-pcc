#include "VoxelPointCountToVoxelCount.h"

#include <filesystem>
#include <utility>

using namespace std;
using namespace std::filesystem;

namespace jpcc {

//////////////////////////////////////////////////////////////////////////////////////////////
VoxelPointCountToVoxelCount::VoxelPointCountToVoxelCount(std::string filename, double resolution) :
    Analyzer(std::move(filename)), octreeCounter_(resolution) {}

//////////////////////////////////////////////////////////////////////////////////////////////
bool VoxelPointCountToVoxelCount::exists() { return filesystem::exists(path(filename_)); }

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelPointCountToVoxelCount::compute(FrameConstPtr background, FrameConstPtr dynamic, FrameConstPtr other) {
  octreeCounter_.addFrame(0, background);
  octreeCounter_.addFrame(1, dynamic);
  octreeCounter_.addFrame(2, other);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelPointCountToVoxelCount::finalCompute() {
  octree::OctreeCounter<PointNormal, 3>::CountMap countMap = octreeCounter_.getOccupancyCountToVoxelCount();
  std::ofstream                                   ofs(filename_);
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
}

}  // namespace jpcc
