#include <jpcc/io/PlyReader.h>

#include <jpcc/io/PlyIO.h>

namespace jpcc::io {

using namespace std;

PlyReader::PlyReader(DatasetReaderParameter param, DatasetParameter datasetParam) :
    DatasetReaderBase(std::move(param), std::move(datasetParam)) {}

void PlyReader::loadAll(size_t                     startFrameNumber,
                        size_t                     groupOfFramesSize,
                        std::vector<GroupOfFrame>& sources,
                        bool                       parallel) {
  sources.resize(1);
  size_t endFrameNumber =
      min(startFrameNumber + groupOfFramesSize, datasetParam_.getStartFrameNumbers() + datasetParam_.getFrameCounts());
  if (startFrameNumber >= endFrameNumber) { return; }

  loadPly(sources.at(0), datasetParam_.getFilePath(), startFrameNumber, endFrameNumber, parallel);
  for (size_t i = startFrameNumber; i < endFrameNumber; i++) {
    cout << datasetParam_.getFilePath() << ":" << i << " " << *sources.at(0).at(i - startFrameNumber) << endl;
  }
}

void PlyReader::load(size_t datasetIndex, size_t startFrameNumber, size_t groupOfFramesSize, GroupOfFrame& frames) {
  BOOST_THROW_EXCEPTION(logic_error(string("Not Implemented ")));
}

}  // namespace jpcc::io