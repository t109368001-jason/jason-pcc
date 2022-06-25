#include <jpcc/io/PlyReader.h>

#include <algorithm>

#include <pcl/console/print.h>

#include <jpcc/io/DatasetParameter.h>
#include <jpcc/io/DatasetReaderParameter.h>
#include <jpcc/io/PlyIO.h>

namespace jpcc::io {

//////////////////////////////////////////////////////////////////////////////////////////////
PlyReader::PlyReader(DatasetReaderParameter param, DatasetParameter datasetParam) :
    DatasetReader(std::move(param), std::move(datasetParam)) {
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void PlyReader::load(const size_t  datasetIndex,
                     const size_t  startFrameNumber,
                     const size_t  groupOfFramesSize,
                     GroupOfFrame& frames) {
  const size_t endFrameNumber =
      std::min(startFrameNumber + groupOfFramesSize, this->datasetParam_.getStartFrameNumbers(datasetIndex) +
                                                         this->datasetParam_.getFrameCounts(datasetIndex));
  if (startFrameNumber >= endFrameNumber) { return; }

  loadPly(frames, this->datasetParam_.getFilePath(datasetIndex), startFrameNumber, endFrameNumber, true);
  for (size_t i = startFrameNumber; i < endFrameNumber; i++) {
    std::cout << this->datasetParam_.getFilePath(datasetIndex) << ":" << i << " " << *frames.at(i - startFrameNumber)
              << std::endl;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void PlyReader::open_(size_t JPCC_NOT_USED(datasetIndex), size_t JPCC_NOT_USED(startFrameNumber)) {}

//////////////////////////////////////////////////////////////////////////////////////////////
bool PlyReader::isOpen_(size_t JPCC_NOT_USED(datasetIndex)) const { return true; }

}  // namespace jpcc::io