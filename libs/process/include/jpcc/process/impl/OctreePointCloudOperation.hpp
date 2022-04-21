#pragma once

#include "jpcc/process/OctreePointCloudOperation.h"

namespace jpcc::process {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, bool quantize>
OctreePointCloudOperation<PointT, quantize>::OctreePointCloudOperation(double resolution) :
    OctreePointCloudT(resolution) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, bool quantize>
void OctreePointCloudOperation<PointT, quantize>::setSource(FramePtr source) {
  source_ = source;
  this->switchBuffers(0);
  this->deleteBuffer(0);
  this->setInputCloud(source_);
  this->addPointsFromInputCloud();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, bool quantize>
void OctreePointCloudOperation<PointT, quantize>::setTarget(FramePtr target) {
  target_ = target;
  this->switchBuffers(1);
  this->deleteBuffer(1);
  this->setInputCloud(target_);
  this->addPointsFromInputCloud();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, bool quantize>
FramePtr<PointT> OctreePointCloudOperation<PointT, quantize>::targetAndNotSource() {
  auto indices = make_shared<Indices>();
  this->switchBuffers(1);
  this->getIndicesByFilter(
      [](const typename OctreeNBufT::BufferPattern& bufferPattern) { return !bufferPattern.test(0); }, *indices);

  auto output = make_shared<Frame>();

  process::split<PointT>(target_, indices, output, nullptr);
  return output;
}

}  // namespace jpcc::process