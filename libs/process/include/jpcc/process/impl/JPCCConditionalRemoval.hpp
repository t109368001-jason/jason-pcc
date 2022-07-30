#include <utility>

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
void JPCCConditionalRemoval<PointT>::applyFilter(Indices& indices) {
  indices.resize(this->indices_->size());
  this->removed_indices_->resize(this->indices_->size());
  int outputIndex = 0;
  int removeIndex = 0;

  for (const auto& index : (*this->indices_)) {
    const PointT& point   = this->input_->at(index);
    bool          predict = param_.condition.predict(point);
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