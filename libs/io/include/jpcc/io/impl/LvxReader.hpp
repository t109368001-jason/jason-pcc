#pragma once

#include <fstream>
#include <utility>

#include <lds.h>

namespace jpcc::io {

using namespace std;
using namespace std::placeholders;
using namespace livox_ros;
using namespace jpcc;

template <typename PointT>
LvxReader<PointT>::LvxReader(DatasetReaderParameter param, DatasetParameter datasetParam) :
    DatasetReaderBase<PointT>(std::move(param), std::move(datasetParam)) {
  assert(DatasetReaderBase<PointT>::datasetParam_.type == "lvx");
  if (DatasetReaderBase<PointT>::datasetParam_.sensor == "mid-100") {
    capacity_ = (size_t)((double)(100000 * 3) / DatasetReaderBase<PointT>::param_.frequency * 1.05);
  } else if (DatasetReaderBase<PointT>::datasetParam_.sensor == "mid-70") {
    capacity_ = (size_t)((double)(100000) / DatasetReaderBase<PointT>::param_.frequency * 1.05);
  } else {
    throw logic_error("Not support dataset.sensor " + DatasetReaderBase<PointT>::datasetParam_.sensor);
  }
  lvxs_.resize(DatasetReaderBase<PointT>::datasetParam_.count());
  for_each(DatasetReaderBase<PointT>::datasetIndices_.begin(), DatasetReaderBase<PointT>::datasetIndices_.end(),
           [this](auto&& PH1) { open_(std::forward<decltype(PH1)>(PH1), 0); });
}

template <typename PointT>
void LvxReader<PointT>::open_(const size_t datasetIndex, const size_t startFrameNumber) {
  if (lvxs_.at(datasetIndex) && DatasetReaderBase<PointT>::currentFrameNumbers_.at(datasetIndex) <= startFrameNumber) {
    return;
  }
  if (lvxs_.at(datasetIndex)) { close_(datasetIndex); }
  string                    lvxPath = DatasetReaderBase<PointT>::datasetParam_.getFilePath(datasetIndex);
  shared_ptr<LvxFileHandle> lvx(new LvxFileHandle());
  assert(lvx->Open(lvxPath.c_str(), std::ios::in) == 0);

  assert(lvx->GetFileVersion() == kLvxFileV1);
  assert(lvx->GetDeviceCount() != 0);
  assert(lvx->GetDeviceCount() < kMaxSourceLidar);
  lvxs_.at(datasetIndex) = lvx;
}

template <typename PointT>
bool LvxReader<PointT>::isOpen_(const size_t datasetIndex) {
  return static_cast<bool>(lvxs_.at(datasetIndex));
}

template <typename PointT>
bool LvxReader<PointT>::isEof_(const size_t datasetIndex) {
  return DatasetReaderBase<PointT>::isEof_(datasetIndex) || lvxs_.at(datasetIndex)->GetFileState() == kLvxFileAtEnd;
}

template <typename PointT>
void LvxReader<PointT>::load_(const size_t          datasetIndex,
                              const size_t          startFrameNumber,
                              const size_t          groupOfFramesSize,
                              GroupOfFrame<PointT>& frames) {
  assert(groupOfFramesSize > 0);
  size_t&                   currentFrameNumber = DatasetReaderBase<PointT>::currentFrameNumbers_.at(datasetIndex);
  shared_ptr<LvxFileHandle> lvx                = lvxs_.at(datasetIndex);
  vector<FramePtr<PointT>>& frameBuffer        = DatasetReaderBase<PointT>::frameBuffers_.at(datasetIndex);

  std::vector<int64_t> lastTimestamps(lvx->GetDeviceCount());
  int ret = lvx->parsePacketsOfFrameXYZ([&](int64_t timestampNS, uint8_t deviceIndex, float x, float y, float z) {
    int64_t timestamp = timestampNS / 1000000;
    if (frameBuffer.empty()) {
      // new frame
      FramePtr<PointT> frame(new Frame<PointT>());
      frame->header.stamp = (int64_t)((float)timestamp / DatasetReaderBase<PointT>::param_.interval *
                                      DatasetReaderBase<PointT>::param_.interval);
      frame->reserve(capacity_);
      frameBuffer.push_back(frame);
    }
    int64_t index =
        (timestamp - (int64_t)frameBuffer.front()->header.stamp) / (int64_t)DatasetReaderBase<PointT>::param_.interval;
    if (index < 0) {
      for (int i = -1; i >= index; i--) {
        // new frame
        FramePtr<PointT> frame(new Frame<PointT>());
        frame->header.stamp =
            frameBuffer.front()->header.stamp + (int64_t)(DatasetReaderBase<PointT>::param_.interval * (float)i);
        frame->reserve(capacity_);
        frameBuffer.insert(frameBuffer.begin(), frame);
      }
      index = 0;
    } else if (index >= frameBuffer.size()) {
      for (size_t i = frameBuffer.size(); i <= index; i++) {
        // new frame
        FramePtr<PointT> frame(new Frame<PointT>());
        frame->header.stamp =
            frameBuffer.front()->header.stamp + (int64_t)(DatasetReaderBase<PointT>::param_.interval * (float)i);
        frame->reserve(capacity_);
        frameBuffer.push_back(frame);
      }
    }
    if (x >= DatasetReaderBase<PointT>::param_.epsilon || y >= DatasetReaderBase<PointT>::param_.epsilon ||
        z >= DatasetReaderBase<PointT>::param_.epsilon) {
      // TODO add transform
      // emplace_back points only, improve performance
      // frameBuffer.at(index)->emplace_back(x, y, z);
      frameBuffer.at(index)->points.emplace_back(x, y, z);
    }
    lastTimestamps.at(deviceIndex) = timestamp;
  });
  assert(ret == 0 || ret == kLvxFileAtEnd);

  if (ret == kLvxFileAtEnd) {
    for (const FramePtr<PointT>& frame : frameBuffer) {
      frame->width  = static_cast<std::uint32_t>(frame->size());
      frame->height = 1;
    }
  } else {
    int64_t minLastTimestamp = *std::min_element(lastTimestamps.begin(), lastTimestamps.end());
    for (const FramePtr<PointT>& frame : frameBuffer) {
      if ((frame->header.stamp + (int64_t)DatasetReaderBase<PointT>::param_.interval) > minLastTimestamp) { break; }
      frame->width  = static_cast<std::uint32_t>(frame->size());
      frame->height = 1;
    }
  }
}

template <typename PointT>
void LvxReader<PointT>::close_(const size_t datasetIndex) {
  DatasetReaderBase<PointT>::close_(datasetIndex);
  if (lvxs_.at(datasetIndex)) {
    lvxs_.at(datasetIndex)->CloseLvxFile();
    lvxs_.at(datasetIndex) = nullptr;
  }
}

}  // namespace jpcc::io
