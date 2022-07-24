#include <jpcc/segmentation/OctreeContainerSegmentationGMMCenter.h>

namespace jpcc::segmentation {

//////////////////////////////////////////////////////////////////////////////////////////////
OctreeContainerSegmentationGMMCenter::OctreeContainerSegmentationGMMCenter() :
    OctreeContainerStaticFlag(), OctreeContainerGMM() {}

//////////////////////////////////////////////////////////////////////////////////////////////
void OctreeContainerSegmentationGMMCenter::reset() {
  OctreeContainerStaticFlag::reset();
  OctreeContainerGMM::reset();
}

}  // namespace jpcc::segmentation