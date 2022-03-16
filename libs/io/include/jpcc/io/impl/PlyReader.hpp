#pragma once

#include <pcl/console/print.h>

#include <jpcc/io/PlyIO.h>

namespace jpcc::io {

using namespace std;

template <typename PointT>
PlyReader<PointT>::PlyReader(DatasetReaderParameter param, DatasetParameter datasetParam) :
    DatasetReaderBase<PointT>(std::move(param), std::move(datasetParam)) {
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
}

template <typename PointT>
void PlyReader<PointT>::loadAll(size_t                startFrameNumber,
                                size_t                groupOfFramesSize,
                                GroupOfFrame<PointT>& sources,
                                bool                  parallel) {
  size_t endFrameNumber =
      min(startFrameNumber + groupOfFramesSize, DatasetReaderBase<PointT>::datasetParam_.getStartFrameNumbers() +
                                                    DatasetReaderBase<PointT>::datasetParam_.getFrameCounts());
  if (startFrameNumber >= endFrameNumber) { return; }

  loadPly(sources, DatasetReaderBase<PointT>::datasetParam_.getFilePath(), startFrameNumber, endFrameNumber, parallel);
  for (size_t i = startFrameNumber; i < endFrameNumber; i++) {
    cout << DatasetReaderBase<PointT>::datasetParam_.getFilePath() << ":" << i << " "
         << *sources.at(i - startFrameNumber) << endl;
  }
}

}  // namespace jpcc::io