#include <jpcc/io/DatasetReader.h>

#include <algorithm>
#include <execution>

namespace jpcc::io {

//////////////////////////////////////////////////////////////////////////////////////////////
DatasetReader::DatasetReader(DatasetReaderParameter param, DatasetParameter datasetParam) :
    DatasetReaderBase(param, datasetParam) {}

//////////////////////////////////////////////////////////////////////////////////////////////
void DatasetReader::loadAll(const size_t  startFrameNumber,
                            const size_t  groupOfFramesSize,
                            GroupOfFrame& frames,
                            const bool    parallel) {
  assert(groupOfFramesSize > 0);

  open(startFrameNumber);

  std::vector<GroupOfFrame> sources;
  sources.resize(datasetIndices_.size());
  if (parallel) {
    std::for_each(std::execution::par, datasetIndices_.begin(), datasetIndices_.end(), [&](const size_t& datasetIndex) {
      load(datasetIndex, startFrameNumber, groupOfFramesSize, sources.at(datasetIndex));
    });
  } else {
    std::for_each(datasetIndices_.begin(), datasetIndices_.end(), [&](const size_t& datasetIndex) {
      load(datasetIndex, startFrameNumber, groupOfFramesSize, sources.at(datasetIndex));
    });
  }
  const size_t maxSize =
      std::max_element(sources.begin(), sources.end(), [](const GroupOfFrame& a, const GroupOfFrame& b) {
        return a.size() < b.size();
      })->size();
  frames.resize(maxSize);
  for (size_t i = 0; i < maxSize; i++) {
    FramePtr& frame   = frames.at(i);
    frame             = jpcc::make_shared<Frame>();
    frame->header.seq = startFrameNumber + i;
    for (size_t datasetIndex = 0; datasetIndex < datasetParam_.count(); datasetIndex++) {
      if (i < sources.at(datasetIndex).size()) {
        frame->insert(frame->end(), make_move_iterator(sources.at(datasetIndex).at(i)->begin()),
                      make_move_iterator(sources.at(datasetIndex).at(i)->end()));
      }
    }
    frame->width  = static_cast<uint32_t>(frame->size());
    frame->height = 1;
    std::cout << "reader read "
              << "frameNumber=" << frame->header.seq << ", "
              << "points=" << frame->size() << std::endl;
  }
}

}  // namespace jpcc::io
