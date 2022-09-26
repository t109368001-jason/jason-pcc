#include "VoxelPointCountToVoxelCount.h"

using namespace std;
using namespace std::filesystem;
using namespace jpcc::octree;

namespace jpcc {

//////////////////////////////////////////////////////////////////////////////////////////////
VoxelPointCountToVoxelCount::VoxelPointCountToVoxelCount(const float&  frequency,
                                                         const double& resolution,
                                                         const string& outputDir,
                                                         const string& title) :
    Analyzer(frequency, resolution, outputDir, title), octree_(resolution) {}

//////////////////////////////////////////////////////////////////////////////////////////////
VoxelPointCountToVoxelCount::VoxelPointCountToVoxelCount(const float&  frequency,
                                                         const double& resolution,
                                                         const string& outputDir) :
    VoxelPointCountToVoxelCount(frequency, resolution, outputDir, "VoxelPointCountToVoxelCount") {}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelPointCountToVoxelCount::compute(FrameConstPtr<pcl::PointXYZINormal> background,
                                          FrameConstPtr<pcl::PointXYZINormal> dynamic,
                                          FrameConstPtr<pcl::PointXYZINormal> other) {
  octree_.addFrame(0, background);
  octree_.addFrame(1, dynamic);
  octree_.addFrame(2, other);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelPointCountToVoxelCount::finalCompute() {
  OctreeT::CountMap countMap = octree_.getOccupancyCountToVoxelCount();
  ofstream          ofs(filepath_);
  ofs << "Voxel Point Count"
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
void VoxelPointCountToVoxelCount::getCloud(FramePtr<pcl::PointXYZINormal>& cloud) {
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
      auto i = static_cast<float>(it.getLeafContainer().getCount());
      cloud->points.emplace_back(x, y, z, i);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelPointCountToVoxelCount::reset() {
  Analyzer::reset();
  octree_.deleteTree();
}

}  // namespace jpcc
