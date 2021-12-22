#include <jpcc/io/LvxReader.h>

#include <fstream>
#include <iomanip>
#include <sstream>

#include <lds.h>

namespace jpcc {
namespace io {

using namespace std;
using namespace livox_ros;
using namespace jpcc::common;

LvxReader::LvxReader(const DatasetParameter& datasetParam, const LvxReaderParameter& param) :
    DatasetReader(datasetParam), param_(param) {
  assert(datasetParam_.type == "lvx");
  if (datasetParam_.sensor == "mid-100") {
    capacity_ = 100000 * 3 / param_.frequency * 1.05;
  } else {
    throw logic_error("Not support dataset.sensor " + datasetParam_.sensor);
  }
  currentFrameIndices_.resize(datasetParam_.totalFiles);
  lvxs_.resize(datasetParam_.totalFiles);
  frameBuffers_.resize(datasetParam_.totalFiles);

  for (size_t i = 0; i < datasetParam_.totalFiles; i++) { open(i); }
}

LvxReader::~LvxReader() {
  for (size_t i = 0; i < datasetParam_.totalFiles; i++) { close(i); }
}

void LvxReader::load(const size_t  datasetIndex,
                     GroupOfFrame& frames,
                     const size_t  startFrameIndex,
                     const size_t  groupOfFramesSize,
                     const bool    parallel) {
  open(datasetIndex, startFrameIndex);
  load_(datasetIndex, frames, startFrameIndex, groupOfFramesSize, parallel);
}

const LvxReaderParameter& LvxReader::getLvxReaderParameter() { return param_; }

void LvxReader::open(const size_t datasetIndex, const size_t startFrameIndex) {
  if (lvxs_.at(datasetIndex) && currentFrameIndices_.at(datasetIndex) <= startFrameIndex) { return; }
  if (lvxs_.at(datasetIndex)) { close(datasetIndex); }
  string                    lvxPath = datasetParam_.getFilePath(datasetIndex);
  shared_ptr<LvxFileHandle> lvx(new LvxFileHandle());
  assert(lvx->Open(lvxPath.c_str(), std::ios::in) == 0);

  assert(lvx->GetFileVersion() == kLvxFileV1);
  assert(lvx->GetDeviceCount() != 0);
  assert(lvx->GetDeviceCount() < kMaxSourceLidar);
  lvxs_.at(datasetIndex) = lvx;
}

void LvxReader::close(const size_t datasetIndex) {
  currentFrameIndices_.at(datasetIndex) = 0;
  lvxs_.at(datasetIndex)->CloseLvxFile();
  lvxs_.at(datasetIndex) = nullptr;
  frameBuffers_.at(datasetIndex).clear();
}

void LvxReader::load_(const size_t  datasetIndex,
                      GroupOfFrame& frames,
                      const size_t  startFrameIndex,
                      const size_t  groupOfFramesSize,
                      const bool    parallel) {
  assert(groupOfFramesSize > 0);
  size_t                    endFrameIndex     = startFrameIndex + groupOfFramesSize;
  size_t&                   currentFrameIndex = currentFrameIndices_.at(datasetIndex);
  shared_ptr<LvxFileHandle> lvx               = lvxs_.at(datasetIndex);
  vector<Frame::Ptr>        frameBuffer       = frameBuffers_.at(datasetIndex);

  frames.resize(groupOfFramesSize);
  while (currentFrameIndex < endFrameIndex) {
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
    std::vector<int64_t> lastTimestamps(lvx->GetDeviceCount());
    int ret = lvx->parsePacketsOfFrameXYZ([&](int64_t timestampNS, uint8_t deviceIndex, float x, float y, float z) {
      int64_t timestamp = timestampNS / 1000000;
      if (frameBuffer.empty()) {
        // new frame
        Frame::Ptr frame(new Frame());
        frame->addPointTypes(param_.pointTypes);
        frame->setTimestamp((int64_t)(timestamp / param_.interval) * param_.interval);
        frame->reserve(capacity_);
        frameBuffer.push_back(frame);
      }
      int index = (timestamp - frameBuffer.front()->getTimestamp()) / param_.interval;
      if (index < 0) {
        for (int i = -1; i >= index; i--) {
          // new frame
          Frame::Ptr frame(new Frame());
          frame->addPointTypes(param_.pointTypes);
          frame->setTimestamp(frameBuffer.front()->getTimestamp() + param_.interval * i);
          frame->reserve(capacity_);
          frameBuffer.insert(frameBuffer.begin(), frame);
        }
        index = 0;
      } else if (index >= frameBuffer.size()) {
        for (size_t i = frameBuffer.size(); i <= index; i++) {
          // new frame
          Frame::Ptr frame(new Frame());
          frame->addPointTypes(param_.pointTypes);
          frame->setTimestamp(frameBuffer.front()->getTimestamp() + param_.interval * i);
          frame->reserve(capacity_);
          frameBuffer.push_back(frame);
        }
      }
      if (x >= param_.epsilon || y >= param_.epsilon || z >= param_.epsilon) { frameBuffer.at(index)->add(x, y, z); }
      lastTimestamps.at(deviceIndex) = timestamp;
    });
    if (ret != 0) { break; }
    int64_t minLastTimestamp = *std::min_element(lastTimestamps.begin(), lastTimestamps.end());
    for (Frame::Ptr frame : frameBuffer) {
      if ((frame->getTimestamp() + param_.interval) > minLastTimestamp) { break; }
      frame->setLoaded();
    }
  }
  if (currentFrameIndex >= startFrameIndex) {
    frames.resize(currentFrameIndex - startFrameIndex);
  } else {
    frames.resize(0);
  }
}

}  // namespace io
}  // namespace jpcc
