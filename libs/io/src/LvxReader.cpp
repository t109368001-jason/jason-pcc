#include <jpcc/io/LvxReader.h>

#include <fstream>
#include <stdexcept>
#include <utility>

#include "lds.h"

namespace jpcc::io {

//////////////////////////////////////////////////////////////////////////////////////////////
LvxReader::LvxReader(DatasetReaderParameter param, DatasetParameter datasetParam) :
    DatasetStreamReader(std::move(param), std::move(datasetParam)) {
  THROW_IF_NOT(this->datasetParam_.type == Type::LVX);
  if (this->datasetParam_.sensor == Sensor::MID_100) {
    this->capacity_ = static_cast<size_t>(static_cast<double>(100000 * 3) / this->param_.frequency * 1.05);
  } else if (this->datasetParam_.sensor == Sensor::MID_70) {
    this->capacity_ = static_cast<size_t>(static_cast<double>(100000) / this->param_.frequency * 1.05);
  } else {
    BOOST_THROW_EXCEPTION(std::logic_error("sensor not support"));
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
  if (lvxs_[datasetIndex] && this->currentFrameNumbers_[datasetIndex] <= startFrameNumber) {
    return;
  }
  lvxs_[datasetIndex]       = nullptr;
  const std::string lvxPath = this->datasetParam_.getFilePath(datasetIndex);
  auto*             lvx     = new LvxHandler();

  THROW_IF_NOT(lvx->Open(lvxPath.c_str(), std::ios::in) == 0);
  THROW_IF_NOT(lvx->GetFileVersion() == livox_ros::kLvxFileV1);
  THROW_IF_NOT(lvx->GetDeviceCount() != 0);
  THROW_IF_NOT(lvx->GetDeviceCount() < livox_ros::kMaxSourceLidar);

  lvxs_[datasetIndex].reset(lvx);
  this->currentFrameNumbers_[datasetIndex] = this->datasetParam_.getStartFrameNumbers(datasetIndex);
  this->eof_[datasetIndex]                 = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool LvxReader::isOpen_(const size_t datasetIndex) const {
  return static_cast<bool>(lvxs_[datasetIndex]);
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool LvxReader::isEof_(const size_t datasetIndex) const {
  return DatasetStreamReader::isEof_(datasetIndex) || lvxs_[datasetIndex]->GetFileState() == livox_ros::kLvxFileAtEnd;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void LvxReader::load_(const size_t datasetIndex, const size_t startFrameNumber, const size_t groupOfFramesSize) {
  assert(groupOfFramesSize > 0);
  LvxHandler*            lvx             = lvxs_[datasetIndex].get();
  std::vector<FramePtr>& frameBuffer     = this->frameBuffers_[datasetIndex];
  std::vector<bool>&     finishVector    = this->finishVectors_[datasetIndex];
  std::vector<int64_t>&  timestampVector = this->timestampVectors_[datasetIndex];

  std::vector<int64_t> lastTimestamps(lvx->GetDeviceCount());

  const int ret = lvx->parsePacketsOfFrameXYZ([&](const int64_t timestampNS, const uint8_t deviceIndex, const float x,
                                                  const float y, const float z, const uint8_t reflectivity) {
    if (x < 0.001 && y < 0.001 && z < 0.001) {
      return;
    }
    const int64_t timestamp = timestampNS / 1000000;
    if (frameBuffer.empty()) {
      // new frame
      const auto frame = jpcc::make_shared<Frame>();
      frame->addReflectances();
      frame->reserve(this->capacity_);
      frameBuffer.push_back(frame);
      finishVector.push_back(false);
      timestampVector.push_back(
          (int64_t)(static_cast<float>(timestamp) / this->param_.interval * this->param_.interval));
    }
    int64_t index = (timestamp - (int64_t)timestampVector.front()) / (int64_t)this->param_.interval;
    if (index < 0) {
      for (int i = -1; i >= index; i--) {
        // new frame
        const auto frame = jpcc::make_shared<Frame>();
        frame->addReflectances();
        frame->reserve(this->capacity_);
        frameBuffer.insert(frameBuffer.begin(), frame);
        finishVector.insert(finishVector.begin(), false);
        timestampVector.insert(timestampVector.begin(),
                               timestampVector.front() + (int64_t)(this->param_.interval * static_cast<float>(i)));
      }
      index = 0;
    } else if (index >= frameBuffer.size()) {
      for (size_t i = frameBuffer.size(); i <= index; i++) {
        // new frame
        const auto frame = jpcc::make_shared<Frame>();
        frame->addReflectances();
        frame->reserve(this->capacity_);
        frameBuffer.push_back(frame);
        finishVector.push_back(false);
        timestampVector.push_back(timestampVector.front() + (int64_t)(this->param_.interval * static_cast<float>(i)));
      }
    }

    size_t ii       = frameBuffer[index]->getPointCount() - 1;
    auto&  position = (*frameBuffer[index])[ii];
    position[0]     = PointValueType(x);
    position[1]     = PointValueType(y);
    position[2]     = PointValueType(z);
    if (frameBuffer[index]->hasReflectances()) {
      frameBuffer[index]->getReflectance(ii) = uint16_t(reflectivity);
    }
    lastTimestamps[deviceIndex] = timestamp;
  });
  assert(ret == 0 || ret == livox_ros::kLvxFileAtEnd);
  if (ret == livox_ros::kLvxFileAtEnd) {
    this->eof_[datasetIndex] = true;
  }

  if (ret == livox_ros::kLvxFileAtEnd) {
    for (auto&& i : finishVector) {
      i = true;
    }
  } else {
    const int64_t minLastTimestamp = *min_element(lastTimestamps.begin(), lastTimestamps.end());
    for (size_t i = 0; i < finishVector.size(); i++) {
      if ((timestampVector[i] + (int64_t)this->param_.interval) > minLastTimestamp) {
        break;
      }
      finishVector[i] = true;
    }
  }
}

}  // namespace jpcc::io
