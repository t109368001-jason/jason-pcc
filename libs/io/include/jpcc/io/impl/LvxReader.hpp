#pragma once

#include <fstream>
#include <utility>

#include <lds.h>

namespace jpcc::io {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
LvxReader<PointT>::LvxReader(DatasetReaderParameter param, DatasetParameter datasetParam) :
    DatasetReaderBase<PointT>(std::move(param), std::move(datasetParam)) {
  assert(this->datasetParam_.type == "lvx");
  if (this->datasetParam_.sensor == "mid-100") {
    capacity_ = (size_t)((double)(100000 * 3) / this->param_.frequency * 1.05);
  } else if (this->datasetParam_.sensor == "mid-70") {
    capacity_ = (size_t)((double)(100000) / this->param_.frequency * 1.05);
  } else {
    throw std::logic_error("Not support dataset.sensor " + this->datasetParam_.sensor);
  }
  lvxs_.resize(this->datasetParam_.count());
  for_each(this->datasetIndices_.begin(), this->datasetIndices_.end(),
           [this](auto&& PH1) { open_(std::forward<decltype(PH1)>(PH1), 0); });
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void LvxReader<PointT>::open_(const size_t datasetIndex, const size_t startFrameNumber) {
  if (lvxs_.at(datasetIndex) && this->currentFrameNumbers_.at(datasetIndex) <= startFrameNumber) { return; }
  if (lvxs_.at(datasetIndex)) { close_(datasetIndex); }
  const std::string lvxPath = this->datasetParam_.getFilePath(datasetIndex);
  auto              lvx     = jpcc::make_shared<livox_ros::LvxFileHandle>();
  assert(lvx->Open(lvxPath.c_str(), std::ios::in) == 0);

  assert(lvx->GetFileVersion() == livox_ros::kLvxFileV1);
  assert(lvx->GetDeviceCount() != 0);
  assert(lvx->GetDeviceCount() < livox_ros::kMaxSourceLidar);
  lvxs_.at(datasetIndex)                      = lvx;
  this->currentFrameNumbers_.at(datasetIndex) = this->datasetParam_.getStartFrameNumbers(datasetIndex);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool LvxReader<PointT>::isOpen_(const size_t datasetIndex) const {
  return static_cast<bool>(lvxs_.at(datasetIndex));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool LvxReader<PointT>::isEof_(const size_t datasetIndex) const {
  return DatasetReaderBase<PointT>::isEof_(datasetIndex) ||
         lvxs_.at(datasetIndex)->GetFileState() == livox_ros::kLvxFileAtEnd;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void LvxReader<PointT>::load_(const size_t  datasetIndex,
                              const size_t  startFrameNumber,
                              const size_t  groupOfFramesSize,
                              GroupOfFrame& frames) {
  assert(groupOfFramesSize > 0);
  size_t&                              currentFrameNumber = this->currentFrameNumbers_.at(datasetIndex);
  shared_ptr<livox_ros::LvxFileHandle> lvx                = lvxs_.at(datasetIndex);
  std::vector<FramePtr>&               frameBuffer        = this->frameBuffers_.at(datasetIndex);

  std::vector<int64_t> lastTimestamps(lvx->GetDeviceCount());

  const int ret = lvx->parsePacketsOfFrameXYZ(
      [&](const int64_t timestampNS, const uint8_t deviceIndex, const float x, const float y, const float z) {
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
        // TODO add transform
        // emplace_back points only, improve performance
        // frameBuffer.at(index)->emplace_back(x, y, z);
        frameBuffer.at(index)->points.emplace_back(x, y, z);
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

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void LvxReader<PointT>::close_(const size_t datasetIndex) {
  DatasetReaderBase<PointT>::close_(datasetIndex);
  if (lvxs_.at(datasetIndex)) {
    lvxs_.at(datasetIndex)->CloseLvxFile();
    lvxs_.at(datasetIndex) = nullptr;
  }
}

}  // namespace jpcc::io
