#include <jpcc/io/DatasetReaderBase.h>

#include <algorithm>
#include <exception>
#include <execution>
#include <functional>
#include <utility>

namespace jpcc::io {

using namespace std;
using namespace std::placeholders;
using namespace jpcc;

DatasetReaderBase::DatasetReaderBase(DatasetReaderParameter param, DatasetParameter datasetParam) :
    param_(std::move(param)),
    datasetParam_(std::move(datasetParam)),
    datasetIndices_(datasetParam_.count()),
    currentFrameNumbers_(datasetParam_.count()),
    frameBuffers_(datasetParam_.count()) {
  assert(datasetParam_.count() > 0);
  generate(datasetIndices_.begin(), datasetIndices_.end(), [n = 0]() mutable { return n++; });
}

DatasetReaderBase::~DatasetReaderBase() { close(); }

DatasetReaderParameter DatasetReaderBase::getReaderParameter() { return param_; }

DatasetParameter DatasetReaderBase::getDatasetParameter() { return datasetParam_; }

bool DatasetReaderBase::isOpen() {
  return any_of(datasetIndices_.begin(), datasetIndices_.end(),
                [this](auto&& PH1) { return isOpen_(std::forward<decltype(PH1)>(PH1)); });
}

void DatasetReaderBase::loadAll(const size_t startFrameNumber, const size_t groupOfFramesSize, GroupOfFrame& frames) {
  loadAll(startFrameNumber, groupOfFramesSize, frames, false);
}

void DatasetReaderBase::loadAll(const size_t  startFrameNumber,
                                const size_t  groupOfFramesSize,
                                GroupOfFrame& frames,
                                const bool    parallel) {
  assert(groupOfFramesSize > 0);

  for_each(datasetIndices_.begin(), datasetIndices_.end(),
           [this, startFrameNumber](auto&& PH1) { open_(std::forward<decltype(PH1)>(PH1), startFrameNumber); });

  vector<GroupOfFrame> sources;
  sources.resize(datasetIndices_.size());
  if (parallel) {
    for_each(execution::par, datasetIndices_.begin(), datasetIndices_.end(), [&](size_t datasetIndex) {
      load(datasetIndex, startFrameNumber, groupOfFramesSize, sources.at(datasetIndex));
    });
  } else {
    for_each(datasetIndices_.begin(), datasetIndices_.end(), [&](size_t datasetIndex) {
      load(datasetIndex, startFrameNumber, groupOfFramesSize, sources.at(datasetIndex));
    });
  }
  size_t maxSize = max_element(sources.begin(), sources.end(), [](GroupOfFrame& a, GroupOfFrame& b) {
                     return a.size() < b.size();
                   })->size();
  frames.resize(maxSize);
  for (size_t i = 0; i < maxSize; i++) {
    Frame::Ptr& frame = frames.at(i);
    frame.reset(new Frame());
    frame->header.seq = startFrameNumber + i;
    for (size_t datasetIndex = 0; datasetIndex < datasetParam_.count(); datasetIndex++) {
      if (i < sources.at(datasetIndex).size()) {
        frame->insert(frame->end(), make_move_iterator(sources.at(datasetIndex).at(i)->begin()),
                      make_move_iterator(sources.at(datasetIndex).at(i)->end()));
      }
    }
    frame->width  = static_cast<std::uint32_t>(frame->size());
    frame->height = 1;
  }
}

void DatasetReaderBase::load(const size_t  datasetIndex,
                             const size_t  startFrameNumber,
                             const size_t  groupOfFramesSize,
                             GroupOfFrame& frames) {
  size_t              endFrameNumber     = startFrameNumber + groupOfFramesSize;
  size_t&             currentFrameNumber = currentFrameNumbers_.at(datasetIndex);
  vector<Frame::Ptr>& frameBuffer        = frameBuffers_.at(datasetIndex);

  open_(datasetIndex, startFrameNumber);
  frames.resize(groupOfFramesSize);

  while (!isEof_(datasetIndex) && currentFrameNumber < endFrameNumber) {
    if (!frameBuffer.empty() && frameBuffer.front()->width != 0) {
      if (currentFrameNumber < startFrameNumber) {
        frameBuffer.erase(frameBuffer.begin());
        currentFrameNumber++;
        continue;
      }
      frameBuffer.front()->header.seq                  = currentFrameNumber;
      frames.at(currentFrameNumber - startFrameNumber) = frameBuffer.front();
      cout << datasetParam_.getFilePath(datasetIndex) << ":" << currentFrameNumber << " "
           << *frames.at(currentFrameNumber - startFrameNumber) << endl;
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

void DatasetReaderBase::close() {
  for_each(datasetIndices_.begin(), datasetIndices_.end(),
           [this](auto&& PH1) { close_(std::forward<decltype(PH1)>(PH1)); });
}

void DatasetReaderBase::open_(const size_t datasetIndex, const size_t startFrameNumber) {
  if (isOpen_(datasetIndex) && currentFrameNumbers_.at(datasetIndex) <= startFrameNumber) {
    return;
  } else if (isOpen_(datasetIndex)) {
  }
}

bool DatasetReaderBase::isOpen_(const size_t datasetIndex) { return false; }

bool DatasetReaderBase::isEof_(const size_t datasetIndex) {
  return !isOpen() || (currentFrameNumbers_.at(datasetIndex) >= datasetParam_.frameCounts.at(datasetIndex));
}

void DatasetReaderBase::load_(const size_t  datasetIndex,
                              const size_t  startFrameNumber,
                              const size_t  groupOfFramesSize,
                              GroupOfFrame& frames) {
  BOOST_THROW_EXCEPTION(logic_error(string("Not Implemented ")));
}

void DatasetReaderBase::close_(const size_t datasetIndex) {
  currentFrameNumbers_.at(datasetIndex) = 0;
  frameBuffers_.at(datasetIndex).clear();
}

}  // namespace jpcc::io
