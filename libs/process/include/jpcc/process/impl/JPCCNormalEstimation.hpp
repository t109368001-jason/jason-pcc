#include <algorithm>
#include <execution>
#include <utility>

namespace jpcc::process {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointIn, typename PointOut>
JPCCNormalEstimation<PointIn, PointOut>::JPCCNormalEstimation(JPCCNormalEstimationParameter param) :
    param_(std::move(param)) {
  if (!param_.enable) { BOOST_THROW_EXCEPTION(std::logic_error(std::string("Normal Estimation not enabled"))); }
  normalEstimation_.setRadiusSearch(param_.radiusSearch);
  normalEstimation_.setKSearch(param_.kSearch);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointIn, typename PointOut>
void JPCCNormalEstimation<PointIn, PointOut>::computeInPlace(FramePtr<PointIn>& frame) const {
  if constexpr (!pcl::traits::has_normal_v<PointIn>) {
    static_assert(dependent_false_v<PointIn>, "invalid template type");
  }
  NormalEstimation ne(normalEstimation_);
  ne.setInputCloud(frame);
  ne.compute(*frame);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointIn, typename PointOut>
FramePtr<PointOut> JPCCNormalEstimation<PointIn, PointOut>::compute(FramePtr<PointIn>& frame) const {
  auto             output = jpcc::make_shared<jpcc::Frame<PointOut>>();
  NormalEstimation ne(normalEstimation_);
  ne.setInputCloud(frame);
  ne.compute(*output);
  return output;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointIn, typename PointOut>
void JPCCNormalEstimation<PointIn, PointOut>::computeInPlaceAll(GroupOfFrame<PointIn>& frames, bool parallel) const {
  if constexpr (!pcl::traits::has_normal_v<PointIn>) {
    static_assert(dependent_false_v<PointIn>, "invalid template type");
  }
  if (parallel) {
    std::for_each(std::execution::par_unseq, frames.begin(), frames.end(),
                  [this](FramePtr<PointIn>& frame) { this->computeInPlace(frame); });
  } else {
    std::for_each(frames.begin(), frames.end(), [this](FramePtr<PointIn>& frame) { this->computeInPlace(frame); });
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointIn, typename PointOut>
GroupOfFrame<PointOut> JPCCNormalEstimation<PointIn, PointOut>::computeAll(GroupOfFrame<PointIn>& frames,
                                                                           bool                   parallel) const {
  jpcc::GroupOfFrame<PointOut> outputs;
  outputs.resize(frames.size());
  if (parallel) {
    std::transform(std::execution::par_unseq, frames.begin(), frames.end(), outputs.begin(),
                   [this](FramePtr<PointIn>& frame) { return this->compute(frame); });
  } else {
    std::transform(frames.begin(), frames.end(), outputs.begin(),
                   [this](FramePtr<PointIn>& frame) { return this->compute(frame); });
  }
  return outputs;
}

}  // namespace jpcc::process