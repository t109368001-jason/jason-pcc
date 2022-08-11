#include <execution>

#include <jpcc/segmentation/JPCCSegmentationOPCGMMCenter.h>
#include <jpcc/segmentation/OctreeContainerSegmentationGMMCenter.h>
#include <jpcc/segmentation/OctreeContainerSegmentationGMM2LCenter.h>

namespace jpcc::segmentation {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
typename JPCCSegmentation<PointT>::Ptr JPCCSegmentationAdapter::build(const JPCCSegmentationParameter& parameter,
                                                                      const int startFrameNumber) {
  if (parameter.type == SegmentationType::GMM && parameter.staticPointType == StaticPointType::CENTER) {
    return jpcc::make_shared<JPCCSegmentationOPCGMMCenter<PointT, OctreeContainerSegmentationGMMCenter<PointT>>>(
        parameter, startFrameNumber);
  } else if (parameter.type == SegmentationType::GMM_2L && parameter.staticPointType == StaticPointType::CENTER) {
    return jpcc::make_shared<JPCCSegmentationOPCGMMCenter<PointT, OctreeContainerSegmentationGMM2LCenter<PointT>>>(
        parameter, startFrameNumber);
  } else {
    BOOST_THROW_EXCEPTION(std::logic_error("unsupported staticPointType"));
  }
}

}  // namespace jpcc::segmentation