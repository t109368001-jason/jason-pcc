#include <jpcc/io/DatasetReader.h>

#include <algorithm>
#include <execution>
#include <utility>

namespace jpcc::io {

//////////////////////////////////////////////////////////////////////////////////////////////
DatasetReader::DatasetReader(DatasetReaderParameter param, DatasetParameter datasetParam) :
    DatasetReaderBase(std::move(param), std::move(datasetParam)) {}

//////////////////////////////////////////////////////////////////////////////////////////////
void DatasetReader::loadAll(const size_t  startFrameNumber,
                            const size_t  groupOfFramesSize,
                            GroupOfFrame& frames,
                            const bool    parallel) {
  assert(groupOfFramesSize > 0);

  open(startFrameNumber);

  std::vector<GroupOfFrame> sources;
  sources.resize(datasetIndices_.size());
  if (!parallel) {
    std::for_each(datasetIndices_.begin(), datasetIndices_.end(), [&](const size_t& datasetIndex) {
      load(datasetIndex, startFrameNumber, groupOfFramesSize, sources[datasetIndex], false);
    });
  } else {
    std::for_each(std::execution::par, datasetIndices_.begin(), datasetIndices_.end(), [&](const size_t& datasetIndex) {
      load(datasetIndex, startFrameNumber, groupOfFramesSize, sources[datasetIndex], false);
    });
  }
  const size_t minSize =
      std::min_element(sources.begin(), sources.end(), [](const GroupOfFrame& a, const GroupOfFrame& b) {
        return a.size() < b.size();
      })->size();
  frames.resize(minSize);
  for (Index i = 0; i < minSize; i++) {
    FramePtr& frame         = frames[i];
    frame                   = jpcc::make_shared<Frame>();
    frame->getFrameNumber() = Index(startFrameNumber) + i;
    size_t size             = 0;
    for (size_t datasetIndex = 0; datasetIndex < datasetParam_.count(); datasetIndex++) {
      if (i < sources[datasetIndex].size()) { size += sources[datasetIndex][i]->getPointCount(); }
    }
    frame->reserve(size);
    for (size_t datasetIndex = 0; datasetIndex < datasetParam_.count(); datasetIndex++) {
      if (i < sources[datasetIndex].size()) { frame->append(*sources[datasetIndex][i]); }
    }
    std::cout << "reader read "
              << "frameNumber=" << frame->getFrameNumber() << ", "
              << "points=" << frame->getPointCount() << std::endl;
  }
}

}  // namespace jpcc::io
