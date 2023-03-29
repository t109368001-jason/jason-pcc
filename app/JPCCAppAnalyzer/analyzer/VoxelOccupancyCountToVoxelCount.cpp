#include "VoxelOccupancyCountToVoxelCount.h"

#include <pcl/io/io.h>

#include <jpcc/process/Process.h>

using namespace std;
using namespace std::filesystem;
using namespace jpcc::octree;

namespace jpcc {

//////////////////////////////////////////////////////////////////////////////////////////////
VoxelOccupancyCountToVoxelCount::VoxelOccupancyCountToVoxelCount(const float&  frequency,
                                                                 const double& resolution,
                                                                 const string& outputDir) :
    VoxelPointCountToVoxelCount(frequency, resolution, outputDir, "VoxelOccupancyCountToVoxelCount") {
}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelOccupancyCountToVoxelCount::compute(PclFrameConstPtr<PointAnalyzer> background,
                                              PclFrameConstPtr<PointAnalyzer> dynamic,
                                              PclFrameConstPtr<PointAnalyzer> other) {
  auto quantizedBackground = jpcc::make_shared<PclFrame<PointAnalyzer>>();
  auto quantizedDynamic    = jpcc::make_shared<PclFrame<PointAnalyzer>>();
  auto quantizedOther      = jpcc::make_shared<PclFrame<PointAnalyzer>>();

  pcl::copyPointCloud(*background, *quantizedBackground);
  pcl::copyPointCloud(*dynamic, *quantizedDynamic);
  pcl::copyPointCloud(*other, *quantizedOther);

  auto _quantizedBackground = make_shared<Frame>();
  auto _quantizedDynamic    = make_shared<Frame>();
  auto _quantizedOther      = make_shared<Frame>();
  _quantizedBackground->fromPcl<PointAnalyzer>(quantizedBackground);
  _quantizedDynamic->fromPcl<PointAnalyzer>(quantizedDynamic);
  _quantizedOther->fromPcl<PointAnalyzer>(quantizedOther);
  process::quantize(_quantizedBackground, resolution_);
  process::quantize(_quantizedDynamic, resolution_);
  process::quantize(_quantizedOther, resolution_);

  quantizedBackground = _quantizedBackground->toPcl<PointAnalyzer>();
  quantizedDynamic    = _quantizedDynamic->toPcl<PointAnalyzer>();
  quantizedOther      = _quantizedOther->toPcl<PointAnalyzer>();

  VoxelPointCountToVoxelCount::compute(quantizedBackground, quantizedDynamic, quantizedOther);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelOccupancyCountToVoxelCount::finalCompute() {
  octree::OctreeCounter<3>::CountMap countMap = octree_.getOccupancyCountToVoxelCount();
  ofstream                           ofs(filepath_);
  ofs << "Voxel Occupancy Count"
      << ","
      << "Voxel Count (Background)"
      << ","
      << "Voxel Count (Dynamic)"
      << ","
      << "Voxel Count (Other)" << endl;
  for (const auto& [occupancyCount, countArray] : countMap) {
    ofs << occupancyCount << "," << countArray[0] << "," << countArray[1] << "," << countArray[2] << endl;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelOccupancyCountToVoxelCount::getCloud(PclFramePtr<PointAnalyzer>& cloud) {
  double min_x_, min_y_, min_z_, max_x_, max_y_, max_z_;
  octree_.getBoundingBox(min_x_, min_y_, min_z_, max_x_, max_y_, max_z_);

  cloud = jpcc::make_shared<PclFrame<PointAnalyzer>>();
  for (BufferIndex bufferIndex = 0; bufferIndex < BUFFER_SIZE; bufferIndex++) {
    octree_.switchBuffers(bufferIndex);
    for (auto it = octree_.leaf_depth_begin(), end = octree_.leaf_depth_end(); it != end; ++it) {
      auto x =
          static_cast<float>((static_cast<double>(it.getCurrentOctreeKey().x) + 0.5f) * this->resolution_ + min_x_);
      auto y =
          static_cast<float>((static_cast<double>(it.getCurrentOctreeKey().y) + 0.5f) * this->resolution_ + min_y_);
      auto z =
          static_cast<float>((static_cast<double>(it.getCurrentOctreeKey().z) + 0.5f) * this->resolution_ + min_z_);
      auto i = static_cast<float>(it.getLeafContainer().getCount());
      cloud->points.emplace_back(x, y, z, i);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelOccupancyCountToVoxelCount::reset() {
  VoxelPointCountToVoxelCount::reset();
  octree_.deleteTree();
}

}  // namespace jpcc
