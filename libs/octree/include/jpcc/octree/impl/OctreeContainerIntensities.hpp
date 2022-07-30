using namespace std;

namespace jpcc::octree {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
OctreeContainerIntensities<PointT>::OctreeContainerIntensities() : OctreeContainerBase() {
  OctreeContainerIntensities::reset();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void OctreeContainerIntensities<PointT>::reset() {
  intensities_.clear();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void OctreeContainerIntensities<PointT>::addPoint(const PointT& point) {
  if constexpr (pcl::traits::has_intensity_v<PointT>) {
    intensities_.push_back(point.intensity);
  } else {
    static_assert(dependent_false_v<PointT>, "invalid template type");
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
const std::vector<float>& OctreeContainerIntensities<PointT>::getIntensities() const {
  return intensities_;
}

}  // namespace jpcc::octree