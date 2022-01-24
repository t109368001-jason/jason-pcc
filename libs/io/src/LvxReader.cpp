#include <jpcc/io/LvxReader.h>

#include <fstream>
#include <utility>

#include <lds.h>

namespace jpcc::io {

using namespace std;
using namespace std::placeholders;
using namespace livox_ros;
using namespace jpcc;

LvxReader::LvxReader(DatasetReaderParameter param, DatasetParameter datasetParam) :
    DatasetReaderBase(std::move(param), std::move(datasetParam)) {
  assert(datasetParam_.type == "lvx");
  if (datasetParam_.sensor == "mid-100") {
    capacity_ = (size_t)((double)(100000 * 3) / param_.frequency * 1.05);
  } else {
    throw logic_error("Not support dataset.sensor " + datasetParam_.sensor);
  }
  lvxs_.resize(datasetParam_.totalFiles);
  for_each(datasetIndices_.begin(), datasetIndices_.end(),
           [this](auto&& PH1) { open_(std::forward<decltype(PH1)>(PH1), 0); });
}

void LvxReader::open_(const size_t datasetIndex, const size_t startFrameNumber) {
  if (lvxs_.at(datasetIndex) && currentFrameNumbers_.at(datasetIndex) <= startFrameNumber) { return; }
  if (lvxs_.at(datasetIndex)) { close_(datasetIndex); }
  string                    lvxPath = datasetParam_.getFilePath(datasetIndex);
  shared_ptr<LvxFileHandle> lvx(new LvxFileHandle());
  assert(lvx->Open(lvxPath.c_str(), std::ios::in) == 0);

  assert(lvx->GetFileVersion() == kLvxFileV1);
  assert(lvx->GetDeviceCount() != 0);
  assert(lvx->GetDeviceCount() < kMaxSourceLidar);
  lvxs_.at(datasetIndex) = lvx;
}

bool LvxReader::isOpen_(const size_t datasetIndex) { return static_cast<bool>(lvxs_.at(datasetIndex)); }

bool LvxReader::isEof_(const size_t datasetIndex) {
  return DatasetReaderBase::isEof_(datasetIndex) || lvxs_.at(datasetIndex)->Eof();
}

void LvxReader::load_(const size_t  datasetIndex,
                      const size_t  startFrameNumber,
                      const size_t  groupOfFramesSize,
                      GroupOfFrame& frames) {
  assert(groupOfFramesSize > 0);
  size_t&                   currentFrameNumber = currentFrameNumbers_.at(datasetIndex);
  shared_ptr<LvxFileHandle> lvx                = lvxs_.at(datasetIndex);
  vector<Frame::Ptr>&       frameBuffer        = frameBuffers_.at(datasetIndex);

  std::vector<int64_t> lastTimestamps(lvx->GetDeviceCount());
  int ret = lvx->parsePacketsOfFrameXYZ([&](int64_t timestampNS, uint8_t deviceIndex, float x, float y, float z) {
    int64_t timestamp = timestampNS / 1000000;
    if (frameBuffer.empty()) {
      // new frame
      Frame::Ptr frame(new Frame());
      frame->header.stamp = (int64_t)((float)timestamp / param_.interval * param_.interval);
      frame->reserve(capacity_);
      frameBuffer.push_back(frame);
    }
    int64_t index = (timestamp - (int64_t)frameBuffer.front()->header.stamp) / (int64_t)param_.interval;
    if (index < 0) {
      for (int i = -1; i >= index; i--) {
        // new frame
        Frame::Ptr frame(new Frame());
        frame->header.stamp = frameBuffer.front()->header.stamp + (int64_t)(param_.interval * (float)i);
        frame->reserve(capacity_);
        frameBuffer.insert(frameBuffer.begin(), frame);
      }
      index = 0;
    } else if (index >= frameBuffer.size()) {
      for (size_t i = frameBuffer.size(); i <= index; i++) {
        // new frame
        Frame::Ptr frame(new Frame());
        frame->header.stamp = frameBuffer.front()->header.stamp + (int64_t)(param_.interval * (float)i);
        frame->reserve(capacity_);
        frameBuffer.push_back(frame);
      }
    }
    if (x >= param_.epsilon || y >= param_.epsilon || z >= param_.epsilon) {
      frameBuffer.at(index)->points.emplace_back(x, y, z);
    }
    lastTimestamps.at(deviceIndex) = timestamp;
  });
  if (ret != 0) { assert(ret == kLvxFileAtEnd); }
  int64_t minLastTimestamp = *std::min_element(lastTimestamps.begin(), lastTimestamps.end());
  for (const Frame::Ptr& frame : frameBuffer) {
    if ((frame->header.stamp + (int64_t)param_.interval) > minLastTimestamp) { break; }
    frame->width  = static_cast<std::uint32_t>(frame->points.size());
    frame->height = 1;
  }
}

void LvxReader::close_(const size_t datasetIndex) {
  DatasetReaderBase::close_(datasetIndex);
  if (lvxs_.at(datasetIndex)) {
    lvxs_.at(datasetIndex)->CloseLvxFile();
    lvxs_.at(datasetIndex) = nullptr;
  }
}

}  // namespace jpcc::io
