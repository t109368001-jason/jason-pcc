#include "VoxelOccupancyCountToVoxelCount.h"

#include <pcl/io/io.h>

#include <jpcc/process/Process.h>

using namespace std;
using namespace std::filesystem;

namespace jpcc {

//////////////////////////////////////////////////////////////////////////////////////////////
VoxelOccupancyCountToVoxelCount::VoxelOccupancyCountToVoxelCount(const float&  frequency,
                                                                 const double& resolution,
                                                                 const string& outputDir) :
    VoxelPointCountToVoxelCount(frequency, resolution, outputDir, "VoxelOccupancyCountToVoxelCount") {}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelOccupancyCountToVoxelCount::compute(FrameConstPtr background, FrameConstPtr dynamic, FrameConstPtr other) {
  auto quantizedBackground = jpcc::make_shared<Frame>();
  auto quantizedDynamic    = jpcc::make_shared<Frame>();
  auto quantizedOther      = jpcc::make_shared<Frame>();

  pcl::copyPointCloud(*background, *quantizedBackground);
  pcl::copyPointCloud(*dynamic, *quantizedDynamic);
  pcl::copyPointCloud(*other, *quantizedOther);

  process::quantize<PointNormal>(quantizedBackground, resolution_);
  process::quantize<PointNormal>(quantizedDynamic, resolution_);
  process::quantize<PointNormal>(quantizedOther, resolution_);

  VoxelPointCountToVoxelCount::compute(quantizedBackground, quantizedDynamic, quantizedOther);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelOccupancyCountToVoxelCount::finalCompute() {
  octree::OctreeCounter<PointNormal, 3>::CountMap countMap = octreeCounter_.getOccupancyCountToVoxelCount();
  std::ofstream                                   ofs(filepath_);
  ofs << "Voxel Occupancy Count"
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
