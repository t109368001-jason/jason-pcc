#pragma once

#include <pcl/console/print.h>

#include <jpcc/io/PlyIO.h>

namespace jpcc::io {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
PlyReader<PointT>::PlyReader(DatasetReaderParameter param, DatasetParameter datasetParam) :
    DatasetReaderBase<PointT>(std::move(param), std::move(datasetParam)) {
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void PlyReader<PointT>::loadAll(const size_t  startFrameNumber,
                                const size_t  groupOfFramesSize,
                                GroupOfFrame& sources,
                                const bool    parallel) {
  const size_t endFrameNumber =
      std::min(startFrameNumber + groupOfFramesSize,
               this->datasetParam_.getStartFrameNumbers() + this->datasetParam_.getFrameCounts());
  if (startFrameNumber >= endFrameNumber) { return; }

  loadPly(sources, this->datasetParam_.getFilePath(), startFrameNumber, endFrameNumber, parallel);
  for (size_t i = startFrameNumber; i < endFrameNumber; i++) {
    std::cout << this->datasetParam_.getFilePath() << ":" << i << " " << *sources.at(i - startFrameNumber) << std::endl;
  }
}

}  // namespace jpcc::io