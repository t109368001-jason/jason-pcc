#include <jpcc/process/JPCCConditionalRemoval.h>

#include <algorithm>
#include <utility>

namespace jpcc::process {

//////////////////////////////////////////////////////////////////////////////////////////////

JPCCConditionalRemoval::JPCCConditionalRemoval(JPCCConditionalRemovalParameter param, int extract_removed_indices) :
    pcl::FilterIndices<PointXYZINormal>(extract_removed_indices), param_(std::move(param)) {
  this->filter_name_ = __FUNCTION__;
}

//////////////////////////////////////////////////////////////////////////////////////////////

void JPCCConditionalRemoval::applyFilter(Indices& indices) {
  indices.resize(this->indices_->size());
  this->removed_indices_->resize(this->indices_->size());
  int outputIndex = 0;
  int removeIndex = 0;

  for (const auto& index : (*this->indices_)) {
    const PointXYZINormal& point   = this->input_->at(index);
    bool                   predict = param_.condition.predict(point);
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