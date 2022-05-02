#pragma once

#include <utility>

namespace jpcc::process {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCNormalEstimation<PointT>::JPCCNormalEstimation(JPCCNormalEstimationParameter param) : param_(std::move(param)) {
  normalEstimation_.setRadiusSearch(param_.radiusSearch);
  normalEstimation_.setKSearch(param_.kSearch);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCNormalEstimation<PointT>::computeInPlace(FramePtr& frame) const {
  NormalEstimation ne(normalEstimation_);
  ne.setInputCloud(frame);
  ne.compute(*frame);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
jpcc::Frame<PointNormal>::Ptr JPCCNormalEstimation<PointT>::compute(FramePtr& frame) const {
  auto             output = jpcc::make_shared<jpcc::Frame<PointNormal>>();
  NormalEstimation ne(normalEstimation_);
  ne.setInputCloud(frame);
  ne.compute(*output);
  return output;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCNormalEstimation<PointT>::computeInPlaceAll(GroupOfFrame& frames, bool parallel) const {
  if (parallel) {
    std::for_each(std::execution::par_unseq, frames.begin(), frames.end(),
                  [this](FramePtr& frame) { this->computeInPlace(frame); });
  } else {
    std::for_each(frames.begin(), frames.end(), [this](FramePtr& frame) { this->computeInPlace(frame); });
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
jpcc::GroupOfFrame<PointNormal> JPCCNormalEstimation<PointT>::computeAll(GroupOfFrame& frames, bool parallel) const {
  jpcc::GroupOfFrame<PointNormal> outputs;
  outputs.resize(frames.size());
  if (parallel) {
    std::transform(std::execution::par_unseq, frames.begin(), frames.end(), outputs.begin(),
                   [this](FramePtr& frame) { return this->compute(frame); });
  } else {
    std::transform(frames.begin(), frames.end(), outputs.begin(),
                   [this](FramePtr& frame) { return this->compute(frame); });
  }
  return outputs;
}

}  // namespace jpcc::process