#include <algorithm>

#include <pcl/console/print.h>

#include <jpcc/io/DatasetParameter.h>
#include <jpcc/io/DatasetReaderParameter.h>
#include <jpcc/io/PlyIO.h>

namespace jpcc::io {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
PlyReader<PointT>::PlyReader(DatasetReaderParameter param, DatasetParameter datasetParam) :
    DatasetReader<PointT>(std::move(param), std::move(datasetParam)) {
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void PlyReader<PointT>::load(const size_t          datasetIndex,
                             const size_t          startFrameNumber,
                             const size_t          groupOfFramesSize,
                             GroupOfFrame<PointT>& frames,
                             const bool            parallel) {
  const size_t endFrameNumber =
      std::min(startFrameNumber + groupOfFramesSize, this->datasetParam_.getStartFrameNumbers(datasetIndex) +
                                                         this->datasetParam_.getFrameCounts(datasetIndex));
  if (startFrameNumber >= endFrameNumber) { return; }

  loadPly(frames, this->datasetParam_.getFilePath(datasetIndex), startFrameNumber, endFrameNumber, parallel);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void PlyReader<PointT>::open_(size_t JPCC_NOT_USED(datasetIndex), size_t JPCC_NOT_USED(startFrameNumber)) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool PlyReader<PointT>::isOpen_(size_t JPCC_NOT_USED(datasetIndex)) const {
  return true;
}

}  // namespace jpcc::io