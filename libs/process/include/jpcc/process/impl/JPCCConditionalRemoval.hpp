#include <utility>

#pragma once

namespace jpcc::process {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCConditionalRemoval<PointT>::JPCCConditionalRemoval(JPCCConditionalRemovalParameter param,
                                                       int                             extract_removed_indices) :
    pcl::FilterIndices<PointT>(extract_removed_indices), param_(std::move(param)) {
  this->filter_name_ = __FUNCTION__;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCConditionalRemoval<PointT>::applyFilter(pcl::Indices& indices) {
  indices.resize(this->indices_->size());
  this->removed_indices_->resize(this->indices_->size());
  int outputIndex = 0;
  int removeIndex = 0;

  for (const auto& index : (*this->indices_)) {
    bool predict = std::all_of(param_.conditions.begin(), param_.conditions.end(), [&](const Condition& condition) {
      double        val;
      const PointT& point = this->input_->at(index);
      switch (condition.field) {
        case Condition::X: val = point.x; break;
        case Condition::Y: val = point.y; break;
        case Condition::Z: val = point.z; break;
        case Condition::R: val = point.getVector3fMap().norm(); break;
        default: return false;
      }

      switch (condition.operation) {
        case Condition::GT: return val > condition.threshold;
        case Condition::GE: return val >= condition.threshold;
        case Condition::LT: return val < condition.threshold;
        case Condition::LE: return val <= condition.threshold;
        case Condition::EQ: return val == condition.threshold;
        default: return false;
      }
    });
    if (!predict) {
      if (this->extract_removed_indices_) this->removed_indices_->at(removeIndex++) = index;
      continue;
    }
    indices.at(outputIndex++) = index;
  }

  indices.resize(outputIndex);
  this->removed_indices_->resize(removeIndex);
}

}  // namespace jpcc::process