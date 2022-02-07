#include <jpcc/io/PlyReader.h>

#include <jpcc/io/PlyIO.h>

namespace jpcc::io {

using namespace std;

PlyReader::PlyReader(DatasetReaderParameter param, DatasetParameter datasetParam) :
    DatasetReaderBase(std::move(param), std::move(datasetParam)) {}

void PlyReader::loadAll(size_t startFrameNumber, size_t groupOfFramesSize, GroupOfFrame& sources, bool parallel) {
  size_t endFrameNumber =
      min(startFrameNumber + groupOfFramesSize, datasetParam_.getStartFrameNumbers() + datasetParam_.getFrameCounts());
  if (startFrameNumber >= endFrameNumber) { return; }

  loadPly(sources, datasetParam_.getFilePath(), startFrameNumber, endFrameNumber, parallel);
  for (size_t i = startFrameNumber; i < endFrameNumber; i++) {
    cout << datasetParam_.getFilePath() << ":" << i << " " << *sources.at(i - startFrameNumber) << endl;
  }
}

}  // namespace jpcc::io