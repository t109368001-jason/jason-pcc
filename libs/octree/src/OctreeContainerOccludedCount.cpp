#include <jpcc/octree/OctreeContainerOccludedCount.h>

using namespace std;
using namespace Eigen;

namespace jpcc::octree {

//////////////////////////////////////////////////////////////////////////////////////////////
OctreeContainerOccludedCount::OctreeContainerOccludedCount() : pointBuffer_() {}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerOccludedCount::reset() { pointBuffer_.clear(); }

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerOccludedCount::addPoint(const PointNormal& point) { pointBuffer_.push_back(point); }

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerOccludedCount::compute(const Vector3f& min_pt, const Vector3f& max_pt, const size_t quantCount) {
  if (count3D_.empty()) { initCountMatrix(quantCount); }
  for (const PointNormal& point : pointBuffer_) {
    int xIndex = (int)((point.x - min_pt.x()) / (max_pt.x() - min_pt.x()) * (float)quantCount);
    int yIndex = (int)((point.y - min_pt.y()) / (max_pt.y() - min_pt.y()) * (float)quantCount);
    int zIndex = (int)((point.z - min_pt.z()) / (max_pt.z() - min_pt.z()) * (float)quantCount);
    count3D_.at(xIndex).at(yIndex).at(zIndex) = 1;
  }
  pointBuffer_.clear();
}

//////////////////////////////////////////////////////////////////////////////////////////////
float OctreeContainerOccludedCount::getMinimumOccludedPercentage() {
  vector<float> occludedPercentage = {getXYOccludedPercentage(), getXZOccludedPercentage(), getYZOccludedPercentage()};
  return *max_element(occludedPercentage.begin(), occludedPercentage.end());
}

float OctreeContainerOccludedCount::getXYOccludedPercentage() {
  size_t total    = 0;
  size_t occluded = 0;
  for (size_t x = 0; x < count3D_.size(); x++) {  // NOLINT(modernize-loop-convert)
    for (size_t y = 0; y < count3D_.size(); y++) {
      size_t count = 0;
      for (size_t z = 0; z < count3D_.size(); z++) {
        if (count3D_.at(x).at(y).at(z) > 0) { count++; }
      }
      if (count > 0) {
        total += count;
        occluded += (count - 1);
      }
    }
  }
  return (float)occluded / (float)total * 100.0f;
}

//////////////////////////////////////////////////////////////////////////////////////////////
float OctreeContainerOccludedCount::getXZOccludedPercentage() {
  size_t total    = 0;
  size_t occluded = 0;
  for (size_t x = 0; x < count3D_.size(); x++) {  // NOLINT(modernize-loop-convert)
    for (size_t z = 0; z < count3D_.size(); z++) {
      size_t count = 0;
      for (size_t y = 0; y < count3D_.size(); y++) {
        if (count3D_.at(x).at(y).at(z) > 0) { count++; }
      }
      if (count > 0) {
        total += count;
        occluded += (count - 1);
      }
    }
  }
  return (float)occluded / (float)total * 100.0f;
}

//////////////////////////////////////////////////////////////////////////////////////////////
float OctreeContainerOccludedCount::getYZOccludedPercentage() {
  size_t total    = 0;
  size_t occluded = 0;
  for (size_t y = 0; y < count3D_.size(); y++) {
    for (size_t z = 0; z < count3D_.size(); z++) {
      size_t count = 0;
      for (size_t x = 0; x < count3D_.size(); x++) {  // NOLINT(modernize-loop-convert)
        if (count3D_.at(x).at(y).at(z) > 0) { count++; }
      }
      if (count > 0) {
        total += count;
        occluded += (count - 1);
      }
    }
  }
  return (float)occluded / (float)total * 100.0f;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerOccludedCount::initCountMatrix(const size_t quantCount) {
  count3D_.resize(quantCount);
  count3D_.shrink_to_fit();
  for (Count2D& count2D : count3D_) {
    count2D.resize(quantCount);
    count2D.shrink_to_fit();
    for (Count1D& count1D : count2D) {
      count1D.resize(quantCount);
      count1D.shrink_to_fit();
    }
  }
}

}  // namespace jpcc::octree