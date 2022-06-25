#include <jpcc/process/JPCCNormalEstimation.h>

#include <algorithm>
#include <execution>
#include <utility>

namespace jpcc::process {

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCNormalEstimation::JPCCNormalEstimation(JPCCNormalEstimationParameter param) : param_(std::move(param)) {
  if (!param_.enable) { BOOST_THROW_EXCEPTION(std::logic_error(std::string("Normal Estimation not enabled"))); }
  normalEstimation_.setRadiusSearch(param_.radiusSearch);
  normalEstimation_.setKSearch(param_.kSearch);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCNormalEstimation::computeInPlace(FramePtr& frame) const {
  NormalEstimation ne(normalEstimation_);
  ne.setInputCloud(frame);
  ne.compute(*frame);
}

//////////////////////////////////////////////////////////////////////////////////////////////
jpcc::FramePtr JPCCNormalEstimation::compute(FramePtr& frame) const {
  auto             output = jpcc::make_shared<jpcc::Frame>();
  NormalEstimation ne(normalEstimation_);
  ne.setInputCloud(frame);
  ne.compute(*output);
  return output;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCNormalEstimation::computeInPlaceAll(GroupOfFrame& frames, bool parallel) const {
  if (parallel) {
    std::for_each(std::execution::par_unseq, frames.begin(), frames.end(),
                  [this](FramePtr& frame) { this->computeInPlace(frame); });
  } else {
    std::for_each(frames.begin(), frames.end(), [this](FramePtr& frame) { this->computeInPlace(frame); });
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
jpcc::GroupOfFrame JPCCNormalEstimation::computeAll(GroupOfFrame& frames, bool parallel) const {
  jpcc::GroupOfFrame outputs;
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