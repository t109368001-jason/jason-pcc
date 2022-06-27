#include "VoxelPointCountToVoxelCount.h"

using namespace std;
using namespace std::filesystem;

namespace jpcc {

//////////////////////////////////////////////////////////////////////////////////////////////
VoxelPointCountToVoxelCount::VoxelPointCountToVoxelCount(const float&  frequency,
                                                         const double& resolution,
                                                         const string& outputDir,
                                                         const string& title) :
    Analyzer(frequency, resolution, outputDir, title), octreeCounter_(resolution) {}

//////////////////////////////////////////////////////////////////////////////////////////////
VoxelPointCountToVoxelCount::VoxelPointCountToVoxelCount(const float&  frequency,
                                                         const double& resolution,
                                                         const string& outputDir) :
    VoxelPointCountToVoxelCount(frequency, resolution, outputDir, "VoxelPointCountToVoxelCount") {}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelPointCountToVoxelCount::compute(FrameConstPtr background, FrameConstPtr dynamic, FrameConstPtr other) {
  octreeCounter_.addFrame(0, background);
  octreeCounter_.addFrame(1, dynamic);
  octreeCounter_.addFrame(2, other);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelPointCountToVoxelCount::finalCompute() {
  OctreeT::CountMap countMap = octreeCounter_.getOccupancyCountToVoxelCount();
  ofstream          ofs(filepath_);
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
