#include <execution>

#include <jpcc/segmentation/JPCCSegmentationOPCGMMCenter.h>
#include <jpcc/segmentation/OctreeContainerGMM.h>
#include <jpcc/segmentation/OctreeContainerGMM2L.h>
#include <jpcc/segmentation/OctreeContainerSegmentation.h>
#include <jpcc/segmentation/OctreeContainerStaticFlag.h>

namespace jpcc::segmentation {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
typename JPCCSegmentation<PointT>::Ptr JPCCSegmentationAdapter::build(const JPCCSegmentationParameter& parameter,
                                                                      const int startFrameNumber) {
  if (parameter.type == SegmentationType::GMM && parameter.staticPointType == StaticPointType::CENTER) {
    return jpcc::make_shared<JPCCSegmentationOPCGMMCenter<
        PointT, OctreeContainerSegmentation<OctreeContainerGMM<PointT>, OctreeContainerStaticFlag>>  //
                             >(parameter, startFrameNumber);
  } else if (parameter.type == SegmentationType::GMM_2L && parameter.staticPointType == StaticPointType::CENTER) {
    return jpcc::make_shared<JPCCSegmentationOPCGMMCenter<
        PointT, OctreeContainerSegmentation<OctreeContainerGMM2L<PointT>, OctreeContainerStaticFlag>>  //
                             >(parameter, startFrameNumber);
  } else {
    BOOST_THROW_EXCEPTION(std::logic_error("unsupported staticPointType"));
  }
}

}  // namespace jpcc::segmentation