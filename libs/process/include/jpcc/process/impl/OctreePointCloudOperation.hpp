#pragma once

#include <jpcc/process/Process.h>

namespace jpcc::process {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, bool quantize>
OctreePointCloudOperation<PointT, quantize>::OctreePointCloudOperation(double resolution) :
    OctreePointCloudOperation<PointT, quantize>::Base(resolution) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, bool quantize>
void OctreePointCloudOperation<PointT, quantize>::setSource(FramePtr source) {
  source_ = source;
  this->setFrame(0, source_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, bool quantize>
void OctreePointCloudOperation<PointT, quantize>::setTarget(FramePtr target) {
  target_ = target;
  this->setFrame(1, target_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, bool quantize>
FramePtr<PointT> OctreePointCloudOperation<PointT, quantize>::targetAndNotSource() {
  auto indices = make_shared<Indices>();
  this->switchBuffers(1);
  this->getIndicesByFilter(
      [](const typename OctreeBase::BufferPattern& bufferPattern) { return !bufferPattern.test(0); }, *indices);

  auto output = make_shared<Frame>();

  split<PointT>(target_, indices, output, nullptr);
  return output;
}

}  // namespace jpcc::process