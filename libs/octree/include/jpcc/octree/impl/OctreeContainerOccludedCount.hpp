using namespace std;
using namespace Eigen;

namespace jpcc::octree {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
OctreeContainerOccludedCount<PointT>::OctreeContainerOccludedCount() : OctreeContainerBase() {
  OctreeContainerOccludedCount::reset();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void OctreeContainerOccludedCount<PointT>::reset() {
  pointBuffer_.clear();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void OctreeContainerOccludedCount<PointT>::addPoint(const PointT& point) {
  pointBuffer_.push_back(point);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void OctreeContainerOccludedCount<PointT>::compute(const Vector3f& min_pt,
                                                   const Vector3f& max_pt,
                                                   const size_t    quantCount) {
  if (count3D_.empty()) { count3D_.resize(quantCount * quantCount * quantCount); }
  for (const PointT& point : pointBuffer_) {
    auto   xIndex = (size_t)((point.x - min_pt.x()) / (max_pt.x() - min_pt.x()) * (float)quantCount);
    auto   yIndex = (size_t)((point.y - min_pt.y()) / (max_pt.y() - min_pt.y()) * (float)quantCount);
    auto   zIndex = (size_t)((point.z - min_pt.z()) / (max_pt.z() - min_pt.z()) * (float)quantCount);
    size_t index  = xIndex * quantCount * quantCount + yIndex * quantCount + zIndex;

    count3D_.set(index);
  }
  pointBuffer_.clear();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
float OctreeContainerOccludedCount<PointT>::getMinimumOccludedPercentage(const size_t quantCount) {
  vector<float> occludedPercentage = {
      getXYOccludedPercentage(quantCount),  //
      getXZOccludedPercentage(quantCount),  //
      getYZOccludedPercentage(quantCount),  //
  };
  return *max_element(occludedPercentage.begin(), occludedPercentage.end());
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
float OctreeContainerOccludedCount<PointT>::getXYOccludedPercentage(const size_t quantCount) {
  size_t total    = 0;
  size_t occluded = 0;
  for (size_t xIndex = 0; xIndex < quantCount; xIndex++) {
    for (size_t yIndex = 0; yIndex < quantCount; yIndex++) {
      size_t count = 0;
      for (size_t zIndex = 0; zIndex < quantCount; zIndex++) {
        size_t index = xIndex * quantCount * quantCount + yIndex * quantCount + zIndex;
        if (count3D_.test(index)) { count++; }
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
template <typename PointT>
float OctreeContainerOccludedCount<PointT>::getXZOccludedPercentage(const size_t quantCount) {
  size_t total    = 0;
  size_t occluded = 0;
  for (size_t xIndex = 0; xIndex < quantCount; xIndex++) {
    for (size_t zIndex = 0; zIndex < quantCount; zIndex++) {
      size_t count = 0;
      for (size_t yIndex = 0; yIndex < quantCount; yIndex++) {
        size_t index = xIndex * quantCount * quantCount + yIndex * quantCount + zIndex;
        if (count3D_.test(index)) { count++; }
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
template <typename PointT>
float OctreeContainerOccludedCount<PointT>::getYZOccludedPercentage(const size_t quantCount) {
  size_t total    = 0;
  size_t occluded = 0;
  for (size_t yIndex = 0; yIndex < quantCount; yIndex++) {
    for (size_t zIndex = 0; zIndex < quantCount; zIndex++) {
      size_t count = 0;
      for (size_t xIndex = 0; xIndex < quantCount; xIndex++) {
        size_t index = xIndex * quantCount * quantCount + yIndex * quantCount + zIndex;
        if (count3D_.test(index)) { count++; }
      }
      if (count > 0) {
        total += count;
        occluded += (count - 1);
      }
    }
  }
  return (float)occluded / (float)total * 100.0f;
}

}  // namespace jpcc::octree