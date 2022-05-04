#include <jpcc/io/DatasetReaderBase.h>

namespace jpcc::io {

//////////////////////////////////////////////////////////////////////////////////////////////
DatasetReaderBase::DatasetReaderBase(DatasetReaderParameter param, DatasetParameter datasetParam) :
    param_(std::move(param)),
    datasetParam_(std::move(datasetParam)),
    datasetIndices_(datasetParam_.count()),
    currentFrameNumbers_(datasetParam_.count()) {
  assert(datasetParam_.count() > 0);
  generate(datasetIndices_.begin(), datasetIndices_.end(), [n = 0]() mutable { return n++; });
}

//////////////////////////////////////////////////////////////////////////////////////////////
const DatasetReaderParameter& DatasetReaderBase::getReaderParameter() const { return param_; }

//////////////////////////////////////////////////////////////////////////////////////////////
const DatasetParameter& DatasetReaderBase::getDatasetParameter() const { return datasetParam_; }

//////////////////////////////////////////////////////////////////////////////////////////////
void DatasetReaderBase::open(const size_t startFrameNumber) {
  std::for_each(datasetIndices_.begin(), datasetIndices_.end(),
                [this, startFrameNumber](const auto& datasetIndex) { this->open_(datasetIndex, startFrameNumber); });
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool DatasetReaderBase::isOpen() const {
  return std::any_of(datasetIndices_.begin(), datasetIndices_.end(),
                     [this](const auto& datasetIndex) { return this->isOpen_(datasetIndex); });
}

}  // namespace jpcc::io