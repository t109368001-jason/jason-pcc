#include <jpcc/segmentation/JPCCSegmentationBase.h>

using namespace std;

namespace jpcc::segmentation {

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCSegmentationBase::JPCCSegmentationBase(const JPCCSegmentationParameter& parameter) : parameter_(parameter) {}

//////////////////////////////////////////////////////////////////////////////////////////////
size_t JPCCSegmentationBase::getNTrain() const { return parameter_.nTrain; }

}  // namespace jpcc::segmentation