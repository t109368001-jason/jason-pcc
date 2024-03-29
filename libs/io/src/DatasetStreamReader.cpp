#include <jpcc/io/DatasetStreamReader.h>

#include <utility>

namespace jpcc::io {

//////////////////////////////////////////////////////////////////////////////////////////////
DatasetStreamReader::DatasetStreamReader(DatasetReaderParameter param, DatasetParameter datasetParam) :
    DatasetReader(std::move(param), std::move(datasetParam)),
    capacity_(0),
    eof_(this->datasetParam_.count()),
    frameBuffers_(this->datasetParam_.count()),
    finishVectors_(this->datasetParam_.count()),
    timestampVectors_(this->datasetParam_.count()) {
  std::fill(eof_.begin(), eof_.end(), true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void DatasetStreamReader::load(const size_t  datasetIndex,
                               const size_t  startFrameNumber,
                               const size_t  groupOfFramesSize,
                               GroupOfFrame& frames,
                               const bool    parallel) {
  frames.clear();
  const size_t endFrameNumber = std::min(startFrameNumber + groupOfFramesSize,  //
                                         this->datasetParam_.getEndFrameNumbers(datasetIndex));
  if (startFrameNumber >= endFrameNumber) {
    return;
  }

  size_t&                     currentFrameNumber = this->currentFrameNumbers_[datasetIndex];
  GroupOfFrame&               frameBuffer        = frameBuffers_[datasetIndex];
  std::vector<bool>&          finishVector       = finishVectors_[datasetIndex];
  std::vector<int64_t>&       timestampVector    = timestampVectors_[datasetIndex];
  shared_ptr<Eigen::Matrix4f> transform          = this->datasetParam_.getTransforms(datasetIndex);

  open_(datasetIndex, startFrameNumber);
  frames.resize(groupOfFramesSize);

  while (!isEof_(datasetIndex) && currentFrameNumber < endFrameNumber) {
    if (!finishVector.empty() && finishVector.front()) {
      if (currentFrameNumber < startFrameNumber) {
        frameBuffer.erase(frameBuffer.begin());
        finishVector.erase(finishVector.begin());
        timestampVector.erase(timestampVector.begin());
        currentFrameNumber++;
        continue;
      }
      if (transform) {
        // pcl::transformPointCloud(*frameBuffer.front(), *frameBuffer.front(), *transform);
        const Eigen::Matrix<float, 4, 4>& tf = *transform;
        for (size_t i = 0; i < frameBuffer.front()->getPointCount(); i++) {
          auto& position = (*frameBuffer.front())[i];

          const auto x = static_cast<float>(tf(0, 0) * static_cast<float>(position[0]) +
                                            tf(0, 1) * static_cast<float>(position[1]) +
                                            tf(0, 2) * static_cast<float>(position[2]) + tf(0, 3));
          const auto y = static_cast<float>(tf(1, 0) * static_cast<float>(position[0]) +
                                            tf(1, 1) * static_cast<float>(position[1]) +
                                            tf(1, 2) * static_cast<float>(position[2]) + tf(1, 3));
          const auto z = static_cast<float>(tf(2, 0) * static_cast<float>(position[0]) +
                                            tf(2, 1) * static_cast<float>(position[1]) +
                                            tf(2, 2) * static_cast<float>(position[2]) + tf(2, 3));

          position[0] = PointValueType(x);
          position[1] = PointValueType(y);
          position[2] = PointValueType(z);
        }
      }
      frames[currentFrameNumber - startFrameNumber] = frameBuffer.front();
      frameBuffer.erase(frameBuffer.begin());
      finishVector.erase(finishVector.begin());
      timestampVector.erase(timestampVector.begin());
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
bool DatasetStreamReader::isEof_(const size_t datasetIndex) const {
  return !this->isOpen() || eof_[datasetIndex];
}

}  // namespace jpcc::io
