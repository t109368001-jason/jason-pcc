#include <jpcc/segmentation/JPCCSegmentationAdapter.h>

#include <execution>

#include "jpcc/segmentation/JPCCSegmentationNone.h"
#include "jpcc/segmentation/JPCCSegmentationOPCGMMCenter.h"
#include "jpcc/segmentation/OctreeContainerGMM.h"
#include "jpcc/segmentation/OctreeContainerGMM2L.h"
#include "jpcc/segmentation/OctreeContainerSegmentation.h"
#include "jpcc/segmentation/OctreeContainerStaticFlag.h"

namespace jpcc::segmentation {

//////////////////////////////////////////////////////////////////////////////////////////////
typename JPCCSegmentation::Ptr JPCCSegmentationAdapter::build(const JPCCSegmentationParameter& parameter,
                                                              const int                        startFrameNumber) {
  if (parameter.type == SegmentationType::NONE) {
    return jpcc::make_shared<JPCCSegmentationNone>(parameter, startFrameNumber);
  } else if (parameter.type == SegmentationType::GMM && parameter.staticPointType == StaticPointType::CENTER) {
    return jpcc::make_shared<
        JPCCSegmentationOPCGMMCenter<OctreeContainerSegmentation<OctreeContainerGMM, OctreeContainerStaticFlag>>  //
        >(parameter, startFrameNumber);
  } else if (parameter.type == SegmentationType::GMM_2L && parameter.staticPointType == StaticPointType::CENTER) {
    return jpcc::make_shared<
        JPCCSegmentationOPCGMMCenter<OctreeContainerSegmentation<OctreeContainerGMM2L, OctreeContainerStaticFlag>>  //
        >(parameter, startFrameNumber);
  } else {
    BOOST_THROW_EXCEPTION(std::logic_error("unsupported staticPointType"));
  }
}

}  // namespace jpcc::segmentation