#pragma once

#include <algorithm>
#include <exception>
#include <execution>
#include <functional>
#include <utility>

#include <pcl/common/transforms.h>

namespace jpcc::io {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
DatasetStreamReader<PointT>::DatasetStreamReader(DatasetReaderParameter param, DatasetParameter datasetParam) :
    param_(std::move(param)),
    datasetParam_(std::move(datasetParam)),
    datasetIndices_(datasetParam_.count()),
    currentFrameNumbers_(datasetParam_.count()),
    frameBuffers_(datasetParam_.count()) {
  assert(datasetParam_.count() > 0);
  generate(datasetIndices_.begin(), datasetIndices_.end(), [n = 0]() mutable { return n++; });
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
DatasetStreamReader<PointT>::~DatasetStreamReader() {
  close();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
const DatasetReaderParameter& DatasetStreamReader<PointT>::getReaderParameter() const {
  return param_;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
const DatasetParameter& DatasetStreamReader<PointT>::getDatasetParameter() const {
  return datasetParam_;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool DatasetStreamReader<PointT>::isOpen() const {
  return any_of(datasetIndices_.begin(), datasetIndices_.end(),
                [this](auto&& PH1) { return isOpen_(std::forward<decltype(PH1)>(PH1)); });
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void DatasetStreamReader<PointT>::loadAll(const size_t  startFrameNumber,
                                        const size_t  groupOfFramesSize,
                                        GroupOfFrame& frames) {
  loadAll(startFrameNumber, groupOfFramesSize, frames, false);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void DatasetStreamReader<PointT>::loadAll(const size_t  startFrameNumber,
                                        const size_t  groupOfFramesSize,
                                        GroupOfFrame& frames,
                                        const bool    parallel) {
  assert(groupOfFramesSize > 0);

  for_each(datasetIndices_.begin(), datasetIndices_.end(),
           [this, startFrameNumber](auto&& PH1) { open_(std::forward<decltype(PH1)>(PH1), startFrameNumber); });

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
void DatasetStreamReader<PointT>::close() {
  for_each(datasetIndices_.begin(), datasetIndices_.end(),
           [this](auto&& PH1) { close_(std::forward<decltype(PH1)>(PH1)); });
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void DatasetStreamReader<PointT>::load(const size_t  datasetIndex,
                                     const size_t  startFrameNumber,
                                     const size_t  groupOfFramesSize,
                                     GroupOfFrame& frames) {
  const size_t                endFrameNumber     = startFrameNumber + groupOfFramesSize;
  size_t&                     currentFrameNumber = currentFrameNumbers_.at(datasetIndex);
  GroupOfFrame&               frameBuffer        = frameBuffers_.at(datasetIndex);
  shared_ptr<Eigen::Matrix4f> transform          = datasetParam_.getTransforms(datasetIndex);

  open_(datasetIndex, startFrameNumber);
  frames.resize(groupOfFramesSize);

  while (!isEof_(datasetIndex) && currentFrameNumber < endFrameNumber) {
    if (!frameBuffer.empty() && frameBuffer.front()->width != 0) {
      if (currentFrameNumber < startFrameNumber) {
        frameBuffer.erase(frameBuffer.begin());
        currentFrameNumber++;
        continue;
      }
      if (transform) { pcl::transformPointCloud(*frameBuffer.front(), *frameBuffer.front(), *transform); }
      frameBuffer.front()->header.seq                  = currentFrameNumber;
      frames.at(currentFrameNumber - startFrameNumber) = frameBuffer.front();
      std::cout << datasetParam_.getFilePath(datasetIndex) << ":" << currentFrameNumber << " "
                << *frames.at(currentFrameNumber - startFrameNumber) << std::endl;
      frameBuffer.erase(frameBuffer.begin());
      currentFrameNumber++;
    }
    load_(datasetIndex, startFrameNumber, groupOfFramesSize, frames);
  }
  if (currentFrameNumber >= startFrameNumber) {
    frames.resize(currentFrameNumber - startFrameNumber);
  } else {
    frames.resize(0);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void DatasetStreamReader<PointT>::open_(const size_t datasetIndex, const size_t startFrameNumber) {
  if (isOpen_(datasetIndex) && currentFrameNumbers_.at(datasetIndex) <= startFrameNumber) {
    return;
  } else if (isOpen_(datasetIndex)) {
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool DatasetStreamReader<PointT>::isOpen_(const size_t datasetIndex) const {
  return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool DatasetStreamReader<PointT>::isEof_(const size_t datasetIndex) const {
  return !isOpen() || (currentFrameNumbers_.at(datasetIndex) >= datasetParam_.getEndFrameNumbers(datasetIndex));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void DatasetStreamReader<PointT>::load_(const size_t  datasetIndex,
                                      const size_t  startFrameNumber,
                                      const size_t  groupOfFramesSize,
                                      GroupOfFrame& frames) {
  BOOST_THROW_EXCEPTION(std::logic_error(std::string("Not Implemented ")));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void DatasetStreamReader<PointT>::close_(const size_t datasetIndex) {
  currentFrameNumbers_.at(datasetIndex) = datasetParam_.getStartFrameNumbers(datasetIndex);
  frameBuffers_.at(datasetIndex).clear();
}

}  // namespace jpcc::io
