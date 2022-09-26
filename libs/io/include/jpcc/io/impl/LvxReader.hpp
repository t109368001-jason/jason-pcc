#include <fstream>
#include <stdexcept>
#include <utility>

#include "lds.h"

namespace jpcc::io {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
LvxReader<PointT>::LvxReader(DatasetReaderParameter param, DatasetParameter datasetParam) :
    DatasetStreamReader<PointT>(std::move(param), std::move(datasetParam)) {
  THROW_IF_NOT(this->datasetParam_.type == Type::LVX);
  if (this->datasetParam_.sensor == Sensor::MID_100) {
    this->capacity_ = (size_t)((double)(100000 * 3) / this->param_.frequency * 1.05);
  } else if (this->datasetParam_.sensor == Sensor::MID_70) {
    this->capacity_ = (size_t)((double)(100000) / this->param_.frequency * 1.05);
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
template <typename PointT>
void LvxReader<PointT>::open_(const size_t datasetIndex, const size_t startFrameNumber) {
  if (lvxs_[datasetIndex] && this->currentFrameNumbers_[datasetIndex] <= startFrameNumber) { return; }
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
template <typename PointT>
bool LvxReader<PointT>::isOpen_(const size_t datasetIndex) const {
  return static_cast<bool>(lvxs_[datasetIndex]);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool LvxReader<PointT>::isEof_(const size_t datasetIndex) const {
  return DatasetStreamReader<PointT>::isEof_(datasetIndex) ||
         lvxs_[datasetIndex]->GetFileState() == livox_ros::kLvxFileAtEnd;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void LvxReader<PointT>::load_(const size_t datasetIndex,
                              const size_t startFrameNumber,
                              const size_t groupOfFramesSize) {
  assert(groupOfFramesSize > 0);
  LvxHandler*                    lvx         = lvxs_[datasetIndex].get();
  std::vector<FramePtr<PointT>>& frameBuffer = this->frameBuffers_[datasetIndex];

  std::vector<int64_t> lastTimestamps(lvx->GetDeviceCount());

  const int ret = lvx->parsePacketsOfFrameXYZ([&](const int64_t timestampNS, const uint8_t deviceIndex, const float x,
                                                  const float y, const float z, const uint8_t reflectivity) {
    if (x < 0.001 && y < 0.001 && z < 0.001) { return; }
    const int64_t timestamp = timestampNS / 1000000;
    if (frameBuffer.empty()) {
      // new frame
      const auto frame    = jpcc::make_shared<Frame<PointT>>();
      frame->header.stamp = (int64_t)((float)timestamp / this->param_.interval * this->param_.interval);
      frame->reserve(this->capacity_);
      frameBuffer.push_back(frame);
    }
    int64_t index = (timestamp - (int64_t)frameBuffer.front()->header.stamp) / (int64_t)this->param_.interval;
    if (index < 0) {
      for (int i = -1; i >= index; i--) {
        // new frame
        const auto frame    = jpcc::make_shared<Frame<PointT>>();
        frame->header.stamp = frameBuffer.front()->header.stamp + (int64_t)(this->param_.interval * (float)i);
        frame->reserve(this->capacity_);
        frameBuffer.insert(frameBuffer.begin(), frame);
      }
      index = 0;
    } else if (index >= frameBuffer.size()) {
      for (size_t i = frameBuffer.size(); i <= index; i++) {
        // new frame
        const auto frame    = jpcc::make_shared<Frame<PointT>>();
        frame->header.stamp = frameBuffer.front()->header.stamp + (int64_t)(this->param_.interval * (float)i);
        frame->reserve(this->capacity_);
        frameBuffer.push_back(frame);
      }
    }
    // emplace_back points only, improve performance
    // frameBuffer[index]->emplace_back(x, y, z);
    PointT point(x, y, z);
    if constexpr (pcl::traits::has_intensity_v<PointT>) { point.intensity = reflectivity / 255.0; }
    frameBuffer[index]->points.push_back(point);
    lastTimestamps[deviceIndex] = timestamp;
  });
  assert(ret == 0 || ret == livox_ros::kLvxFileAtEnd);
  if (ret == livox_ros::kLvxFileAtEnd) { this->eof_[datasetIndex] = true; }

  if (ret == livox_ros::kLvxFileAtEnd) {
    for (const FramePtr<PointT>& frame : frameBuffer) {
      frame->width  = static_cast<uint32_t>(frame->size());
      frame->height = 1;
    }
  } else {
    const int64_t minLastTimestamp = *min_element(lastTimestamps.begin(), lastTimestamps.end());
    for (const FramePtr<PointT>& frame : frameBuffer) {
      if ((frame->header.stamp + (int64_t)this->param_.interval) > minLastTimestamp) { break; }
      frame->width  = static_cast<uint32_t>(frame->size());
      frame->height = 1;
    }
  }
}

}  // namespace jpcc::io
