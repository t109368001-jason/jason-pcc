#pragma once

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
      if (transform) {
        // pcl::transformPointCloud(*frameBuffer.front(), *frameBuffer.front(), *transform);
        const Eigen::Matrix<float, 4, 4>& tf = *transform;
        std::for_each(frameBuffer.front()->begin(), frameBuffer.front()->end(), [&tf](PointT& p) {
          const float x = static_cast<float>(tf(0, 0) * p.x + tf(0, 1) * p.y + tf(0, 2) * p.z + tf(0, 3));
          const float y = static_cast<float>(tf(1, 0) * p.x + tf(1, 1) * p.y + tf(1, 2) * p.z + tf(1, 3));
          const float z = static_cast<float>(tf(2, 0) * p.x + tf(2, 1) * p.y + tf(2, 2) * p.z + tf(2, 3));

          p.x = x;
          p.y = y;
          p.z = z;
        });
      }
      frameBuffer.front()->header.seq                  = currentFrameNumber;
      frames.at(currentFrameNumber - startFrameNumber) = frameBuffer.front();
      std::cout << this->datasetParam_.getFilePath(datasetIndex) << ":" << currentFrameNumber << " "
                << *frames.at(currentFrameNumber - startFrameNumber) << std::endl;
      frameBuffer.erase(frameBuffer.begin());
      currentFrameNumber++;
    }
    load_(datasetIndex, startFrameNumber, groupOfFramesSize);
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
