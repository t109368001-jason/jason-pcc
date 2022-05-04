#pragma once

#include <exception>
#include <execution>
#include <functional>
#include <utility>

#include <pcl/common/transforms.h>

namespace jpcc::io {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
DatasetStreamReader<PointT>::DatasetStreamReader(DatasetReaderParameter param, DatasetParameter datasetParam) :
    DatasetReader<PointT>::DatasetReader(param, datasetParam), frameBuffers_(this->datasetParam_.count()) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void DatasetStreamReader<PointT>::load(const size_t  datasetIndex,
                                       const size_t  startFrameNumber,
                                       const size_t  groupOfFramesSize,
                                       GroupOfFrame& frames) {
  const size_t                endFrameNumber     = startFrameNumber + groupOfFramesSize;
  size_t&                     currentFrameNumber = this->currentFrameNumbers_.at(datasetIndex);
  GroupOfFrame&               frameBuffer        = frameBuffers_.at(datasetIndex);
  shared_ptr<Eigen::Matrix4f> transform          = this->datasetParam_.getTransforms(datasetIndex);

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
      std::cout << this->datasetParam_.getFilePath(datasetIndex) << ":" << currentFrameNumber << " "
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
bool DatasetStreamReader<PointT>::isEof_(const size_t datasetIndex) const {
  return !this->isOpen() ||
         (this->currentFrameNumbers_.at(datasetIndex) >= this->datasetParam_.getEndFrameNumbers(datasetIndex));
}
}  // namespace jpcc::io
