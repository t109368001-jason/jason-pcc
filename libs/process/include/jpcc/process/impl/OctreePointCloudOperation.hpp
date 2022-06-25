#pragma once

#include <jpcc/process/Process.h>

namespace jpcc::process {

//////////////////////////////////////////////////////////////////////////////////////////////
template <bool quantize>
OctreePointCloudOperation<quantize>::OctreePointCloudOperation(double resolution) : Base(resolution) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <bool quantize>
void OctreePointCloudOperation<quantize>::setSource(FramePtr source) {
  source_ = source;
  this->setFrame(0, source_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <bool quantize>
void OctreePointCloudOperation<quantize>::setTarget(FramePtr target) {
  target_ = target;
  this->setFrame(1, target_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <bool quantize>
FramePtr OctreePointCloudOperation<quantize>::targetAndNotSource() {
  auto indices = make_shared<Indices>();
  this->switchBuffers(1);
  this->getIndicesByFilter(
      [](const typename OctreeBase::BufferPattern& bufferPattern) { return !bufferPattern.test(0); }, *indices);

  auto output = make_shared<Frame>();

  split(target_, indices, output, nullptr);
  return output;
}

}  // namespace jpcc::process