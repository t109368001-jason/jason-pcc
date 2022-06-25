#include <jpcc/io/LvxReader.h>

#include <fstream>
#include <stdexcept>
#include <utility>

#include <lds.h>

namespace jpcc::io {

//////////////////////////////////////////////////////////////////////////////////////////////
LvxReader::LvxReader(DatasetReaderParameter param, DatasetParameter datasetParam) :
    DatasetStreamReader(std::move(param), std::move(datasetParam)) {
  assert(this->datasetParam_.type == "lvx");
  if (this->datasetParam_.sensor == "mid-100") {
    capacity_ = (size_t)((double)(100000 * 3) / this->param_.frequency * 1.05);
  } else if (this->datasetParam_.sensor == "mid-70") {
    capacity_ = (size_t)((double)(100000) / this->param_.frequency * 1.05);
  } else {
    throw std::logic_error("Not support dataset.sensor " + this->datasetParam_.sensor);
  }
  for (size_t i = 0; i < this->datasetParam_.count(); i++) {
    lvxs_.emplace_back(nullptr, [](LvxHandler* ptr) {
      ptr->CloseLvxFile();
      delete ptr;
    });
  }
  std::for_each(this->datasetIndices_.begin(), this->datasetIndices_.end(),
                [this](const size_t& datasetIndex) { open_(datasetIndex, 0); });
}

//////////////////////////////////////////////////////////////////////////////////////////////
void LvxReader::open_(const size_t datasetIndex, const size_t startFrameNumber) {
  if (lvxs_.at(datasetIndex) && this->currentFrameNumbers_.at(datasetIndex) <= startFrameNumber) { return; }
  lvxs_.at(datasetIndex)    = nullptr;
  const std::string lvxPath = this->datasetParam_.getFilePath(datasetIndex);
  auto*             lvx     = new LvxHandler();

  assert(lvx->Open(lvxPath.c_str(), std::ios::in) == 0);
  assert(lvx->GetFileVersion() == livox_ros::kLvxFileV1);
  assert(lvx->GetDeviceCount() != 0);
  assert(lvx->GetDeviceCount() < livox_ros::kMaxSourceLidar);

  lvxs_.at(datasetIndex).reset(lvx);
  this->currentFrameNumbers_.at(datasetIndex) = this->datasetParam_.getStartFrameNumbers(datasetIndex);
}

//////////////////////////////////////////////////////////////////////////////////////////////

bool LvxReader::isOpen_(const size_t datasetIndex) const { return static_cast<bool>(lvxs_.at(datasetIndex)); }

//////////////////////////////////////////////////////////////////////////////////////////////

bool LvxReader::isEof_(const size_t datasetIndex) const {
  return DatasetStreamReader::isEof_(datasetIndex) ||
         lvxs_.at(datasetIndex)->GetFileState() == livox_ros::kLvxFileAtEnd;
}

//////////////////////////////////////////////////////////////////////////////////////////////

void LvxReader::load_(const size_t datasetIndex, const size_t startFrameNumber, const size_t groupOfFramesSize) {
  assert(groupOfFramesSize > 0);
  size_t&                currentFrameNumber = this->currentFrameNumbers_.at(datasetIndex);
  LvxHandler*            lvx                = lvxs_.at(datasetIndex).get();
  std::vector<FramePtr>& frameBuffer        = this->frameBuffers_.at(datasetIndex);

  std::vector<int64_t> lastTimestamps(lvx->GetDeviceCount());

  const int ret = lvx->parsePacketsOfFrameXYZ([&](const int64_t timestampNS, const uint8_t deviceIndex, const float x,
                                                  const float y, const float z, const Reflectivity reflectivity) {
    if (x < 0.001 && y < 0.001 && z < 0.001) { return; }
    const int64_t timestamp = timestampNS / 1000000;
    if (frameBuffer.empty()) {
      // new frame
      const auto frame    = jpcc::make_shared<Frame>();
      frame->header.stamp = (int64_t)((float)timestamp / this->param_.interval * this->param_.interval);
      frame->reserve(capacity_);
      frameBuffer.push_back(frame);
    }
    int64_t index = (timestamp - (int64_t)frameBuffer.front()->header.stamp) / (int64_t)this->param_.interval;
    if (index < 0) {
      for (int i = -1; i >= index; i--) {
        // new frame
        const auto frame    = jpcc::make_shared<Frame>();
        frame->header.stamp = frameBuffer.front()->header.stamp + (int64_t)(this->param_.interval * (float)i);
        frame->reserve(capacity_);
        frameBuffer.insert(frameBuffer.begin(), frame);
      }
      index = 0;
    } else if (index >= frameBuffer.size()) {
      for (size_t i = frameBuffer.size(); i <= index; i++) {
        // new frame
        const auto frame    = jpcc::make_shared<Frame>();
        frame->header.stamp = frameBuffer.front()->header.stamp + (int64_t)(this->param_.interval * (float)i);
        frame->reserve(capacity_);
        frameBuffer.push_back(frame);
      }
    }
    // emplace_back points only, improve performance
    // frameBuffer.at(index)->emplace_back(x, y, z);
    PointXYZINormal point(x, y, z);
    point.intensity = reflectivity;
    frameBuffer.at(index)->points.push_back(point);
    lastTimestamps.at(deviceIndex) = timestamp;
  });
  assert(ret == 0 || ret == livox_ros::kLvxFileAtEnd);

  if (ret == livox_ros::kLvxFileAtEnd) {
    for (const FramePtr& frame : frameBuffer) {
      frame->width  = static_cast<uint32_t>(frame->size());
      frame->height = 1;
    }
  } else {
    const int64_t minLastTimestamp = *min_element(lastTimestamps.begin(), lastTimestamps.end());
    for (const FramePtr& frame : frameBuffer) {
      if ((frame->header.stamp + (int64_t)this->param_.interval) > minLastTimestamp) { break; }
      frame->width  = static_cast<uint32_t>(frame->size());
      frame->height = 1;
    }
  }
}

}  // namespace jpcc::io
