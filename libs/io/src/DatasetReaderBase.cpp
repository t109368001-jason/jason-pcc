#include <jpcc/io/DatasetReaderBase.h>

#include <algorithm>
#include <exception>
#include <execution>
#include <functional>

namespace jpcc::io {

using namespace std;
using namespace std::placeholders;
using namespace jpcc::common;

DatasetReaderBase::DatasetReaderBase(DatasetParameter datasetParam) :
    datasetParam_(std::move(datasetParam)),
    datasetIndices_(datasetParam_.totalFiles),
    currentFrameIndices_(datasetParam_.totalFiles),
    frameBuffers_(datasetParam_.totalFiles) {
  assert(datasetParam_.totalFiles > 0);
  generate(datasetIndices_.begin(), datasetIndices_.end(), [n = 0]() mutable { return n++; });
}

DatasetReaderBase::~DatasetReaderBase() { close(); }

bool DatasetReaderBase::isOpen() {
  return any_of(datasetIndices_.begin(), datasetIndices_.end(),
                [this](auto&& PH1) { return isOpen_(std::forward<decltype(PH1)>(PH1)); });
}

void DatasetReaderBase::loadAll(const size_t          startFrameIndex,
                                const size_t          groupOfFramesSize,
                                vector<GroupOfFrame>& sources,
                                const bool            parallel) {
  assert(groupOfFramesSize > 0);

  for_each(datasetIndices_.begin(), datasetIndices_.end(),
           [this, startFrameIndex](auto&& PH1) { open_(std::forward<decltype(PH1)>(PH1), startFrameIndex); });

  sources.resize(datasetIndices_.size());
  if (parallel) {
    for_each(execution::par, datasetIndices_.begin(), datasetIndices_.end(), [&](size_t datasetIndex) {
      load(datasetIndex, startFrameIndex, groupOfFramesSize, sources.at(datasetIndex));
    });
  } else {
    for_each(datasetIndices_.begin(), datasetIndices_.end(), [&](size_t datasetIndex) {
      load(datasetIndex, startFrameIndex, groupOfFramesSize, sources.at(datasetIndex));
    });
  }
}

void DatasetReaderBase::load(const size_t  datasetIndex,
                             const size_t  startFrameIndex,
                             const size_t  groupOfFramesSize,
                             GroupOfFrame& frames) {
  size_t              endFrameIndex     = startFrameIndex + groupOfFramesSize;
  size_t&             currentFrameIndex = currentFrameIndices_.at(datasetIndex);
  vector<Frame::Ptr>& frameBuffer       = frameBuffers_.at(datasetIndex);

  open_(datasetIndex, startFrameIndex);
  frames.resize(groupOfFramesSize);

  while (!isEof_(datasetIndex) && currentFrameIndex < endFrameIndex) {
    if (!frameBuffer.empty() && frameBuffer.front()->isLoaded()) {
      if (currentFrameIndex < startFrameIndex) {
        frameBuffer.erase(frameBuffer.begin());
        currentFrameIndex++;
        continue;
      }
      frames.at(currentFrameIndex - startFrameIndex) = frameBuffer.front();
      cout << datasetParam_.getFilePath(datasetIndex) << ":" << currentFrameIndex << " "
           << *frames.at(currentFrameIndex - startFrameIndex) << endl;
      frameBuffer.erase(frameBuffer.begin());
      currentFrameIndex++;
    }
    load_(datasetIndex, startFrameIndex, groupOfFramesSize, frames);
  }
  if (currentFrameIndex >= startFrameIndex) {
    frames.resize(currentFrameIndex - startFrameIndex);
  } else {
    frames.resize(0);
  }
}

void DatasetReaderBase::close() {
  for_each(datasetIndices_.begin(), datasetIndices_.end(),
           [this](auto&& PH1) { close_(std::forward<decltype(PH1)>(PH1)); });
}

void DatasetReaderBase::open_(const size_t datasetIndex, const size_t startFrameIndex) {
  if (isOpen_(datasetIndex) && currentFrameIndices_.at(datasetIndex) <= startFrameIndex) {
    return;
  } else if (isOpen_(datasetIndex)) {
  }
}

bool DatasetReaderBase::isOpen_(const size_t datasetIndex) { return false; }

bool DatasetReaderBase::isEof_(const size_t datasetIndex) {
  return !isOpen() || (currentFrameIndices_.at(datasetIndex) >= datasetParam_.frameCounts.at(datasetIndex));
}

void DatasetReaderBase::load_(const size_t  datasetIndex,
                              const size_t  startFrameIndex,
                              const size_t  groupOfFramesSize,
                              GroupOfFrame& frames) {
  BOOST_THROW_EXCEPTION(logic_error(string("Not Implemented ")));
}

void DatasetReaderBase::close_(const size_t datasetIndex) {
  currentFrameIndices_.at(datasetIndex) = 0;
  frameBuffers_.at(datasetIndex).clear();
}

const DatasetParameter& DatasetReaderBase::getDatasetParameter() { return datasetParam_; }

}  // namespace jpcc::io
