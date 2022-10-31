#include <jpcc/io/DatasetStreamReader.h>

namespace jpcc::io {

//////////////////////////////////////////////////////////////////////////////////////////////
DatasetStreamReader::DatasetStreamReader(DatasetReaderParameter param, DatasetParameter datasetParam) :
    DatasetReader(param, datasetParam),
    capacity_(0),
    eof_(this->datasetParam_.count()),
    frameBuffers_(this->datasetParam_.count()) {
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
  if (startFrameNumber >= endFrameNumber) { return; }

  size_t&                     currentFrameNumber = this->currentFrameNumbers_[datasetIndex];
  GroupOfFrame&               frameBuffer        = frameBuffers_[datasetIndex];
  std::vector<bool>&          finishVector       = finishVectors_[datasetIndex];
  std::vector<int64_t>&       timestampVector    = timestampVectors_[datasetIndex];
  shared_ptr<Eigen::Matrix4f> transform          = this->datasetParam_.getTransforms(datasetIndex);

  open_(datasetIndex, startFrameNumber);
  frames.resize(groupOfFramesSize);

  while (!isEof_(datasetIndex) && currentFrameNumber < endFrameNumber) {
    if (!frameBuffer.empty() && finishVector.front()) {
      if (currentFrameNumber < startFrameNumber) {
        frameBuffer.erase(frameBuffer.begin());
        currentFrameNumber++;
        continue;
      }
      if (transform) {
        // pcl::transformPointCloud(*frameBuffer.front(), *frameBuffer.front(), *transform);
        const Eigen::Matrix<float, 4, 4>& tf = *transform;
        for (size_t i = 0; i < frameBuffer.front()->getPointCount(); i++) {
          auto& position = (*frameBuffer.front())[i];

          const auto x = static_cast<float>(tf(0, 0) * float(position.x()) + tf(0, 1) * float(position.y()) +
                                            tf(0, 2) * float(position.z()) + tf(0, 3));
          const auto y = static_cast<float>(tf(1, 0) * float(position.x()) + tf(1, 1) * float(position.y()) +
                                            tf(1, 2) * float(position.z()) + tf(1, 3));
          const auto z = static_cast<float>(tf(2, 0) * float(position.x()) + tf(2, 1) * float(position.y()) +
                                            tf(2, 2) * float(position.z()) + tf(2, 3));

          position.x() = int32_t(x);
          position.y() = int32_t(y);
          position.z() = int32_t(z);
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
bool DatasetStreamReader::isEof_(const size_t datasetIndex) const { return !this->isOpen() || eof_[datasetIndex]; }

}  // namespace jpcc::io
