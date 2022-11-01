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

  PclFramePtr<pcl::PointNormal> pclFrame = frame->toPcl<pcl::PointNormal>();
  ne.setInputCloud(pclFrame);
  ne.compute(*pclFrame);

  frame->addNormal();
  for (size_t i = 0; i < frame->getPointCount(); i++) {
    frame->getNormal(i).x() = pclFrame->points[i].normal_x;
    frame->getNormal(i).y() = pclFrame->points[i].normal_y;
    frame->getNormal(i).z() = pclFrame->points[i].normal_z;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCNormalEstimation::computeInPlaceAll(GroupOfFrame& frames, bool parallel) const {
  if (!parallel) {
    for (auto& frame : frames) { this->computeInPlace(frame); }
  } else {
    std::for_each(std::execution::par_unseq, frames.begin(), frames.end(),
                  [this](FramePtr& frame) { this->computeInPlace(frame); });
  }
}

}  // namespace jpcc::process