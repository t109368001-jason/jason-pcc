#pragma once

#include <algorithm>
#include <execution>

namespace jpcc::io {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
DatasetReader<PointT>::DatasetReader(DatasetReaderParameter param, DatasetParameter datasetParam) :
    param_(std::move(param)),
    datasetParam_(std::move(datasetParam)),
    datasetIndices_(datasetParam_.count()),
    currentFrameNumbers_(datasetParam_.count()) {
  assert(datasetParam_.count() > 0);
  generate(datasetIndices_.begin(), datasetIndices_.end(), [n = 0]() mutable { return n++; });
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
DatasetReader<PointT>::~DatasetReader() {
  close();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
const DatasetReaderParameter& DatasetReader<PointT>::getReaderParameter() const {
  return param_;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
const DatasetParameter& DatasetReader<PointT>::getDatasetParameter() const {
  return datasetParam_;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void DatasetReader<PointT>::loadAll(const size_t  startFrameNumber,
                                    const size_t  groupOfFramesSize,
                                    GroupOfFrame& frames) {
  loadAll(startFrameNumber, groupOfFramesSize, frames, false);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void DatasetReader<PointT>::loadAll(const size_t  startFrameNumber,
                                    const size_t  groupOfFramesSize,
                                    GroupOfFrame& frames,
                                    const bool    parallel) {
  assert(groupOfFramesSize > 0);

  open(startFrameNumber);

  std::vector<GroupOfFrame> sources;
  sources.resize(datasetIndices_.size());
  if (parallel) {
    for_each(std::execution::par, datasetIndices_.begin(), datasetIndices_.end(), [&](size_t datasetIndex) {
      load(datasetIndex, startFrameNumber, groupOfFramesSize, sources.at(datasetIndex));
    });
  } else {
    for_each(datasetIndices_.begin(), datasetIndices_.end(), [&](size_t datasetIndex) {
      load(datasetIndex, startFrameNumber, groupOfFramesSize, sources.at(datasetIndex));
    });
  }
  const size_t maxSize = max_element(sources.begin(), sources.end(), [](const GroupOfFrame& a, const GroupOfFrame& b) {
                           return a.size() < b.size();
                         })->size();
  frames.resize(maxSize);
  for (size_t i = 0; i < maxSize; i++) {
    FramePtr& frame   = frames.at(i);
    frame             = jpcc::make_shared<Frame>();
    frame->header.seq = startFrameNumber + i;
    for (size_t datasetIndex = 0; datasetIndex < datasetParam_.count(); datasetIndex++) {
      if (i < sources.at(datasetIndex).size()) {
        frame->insert(frame->end(), make_move_iterator(sources.at(datasetIndex).at(i)->begin()),
                      make_move_iterator(sources.at(datasetIndex).at(i)->end()));
      }
    }
    frame->width  = static_cast<uint32_t>(frame->size());
    frame->height = 1;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void DatasetReader<PointT>::open(const size_t startFrameNumber) {
  std::for_each(datasetIndices_.begin(), datasetIndices_.end(),
                [this, startFrameNumber](const auto& datasetIndex) { open_(datasetIndex, startFrameNumber); });
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool DatasetReader<PointT>::isOpen() const {
  return std::any_of(datasetIndices_.begin(), datasetIndices_.end(),
                     [this](const auto& datasetIndex) { return isOpen_(datasetIndex); });
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void DatasetReader<PointT>::close() {
  std::for_each(datasetIndices_.begin(), datasetIndices_.end(),
                [this](const auto& datasetIndex) { close_(datasetIndex); });
}

}  // namespace jpcc::io
