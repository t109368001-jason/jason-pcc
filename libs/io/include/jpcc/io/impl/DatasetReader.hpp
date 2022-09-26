#include <algorithm>
#include <execution>

namespace jpcc::io {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
DatasetReader<PointT>::DatasetReader(DatasetReaderParameter param, DatasetParameter datasetParam) :
    DatasetReaderBase(param, datasetParam) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void DatasetReader<PointT>::loadAll(const size_t          startFrameNumber,
                                    const size_t          groupOfFramesSize,
                                    GroupOfFrame<PointT>& frames,
                                    const bool            parallel) {
  assert(groupOfFramesSize > 0);

  open(startFrameNumber);

  std::vector<GroupOfFrame<PointT>> sources;
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
      std::min_element(sources.begin(), sources.end(),
                       [](const GroupOfFrame<PointT>& a, const GroupOfFrame<PointT>& b) { return a.size() < b.size(); })
          ->size();
  frames.resize(minSize);
  for (size_t i = 0; i < minSize; i++) {
    FramePtr<PointT>& frame = frames[i];
    frame                   = jpcc::make_shared<Frame<PointT>>();
    frame->header.seq       = startFrameNumber + i;
    size_t size             = 0;
    for (size_t datasetIndex = 0; datasetIndex < datasetParam_.count(); datasetIndex++) {
      if (i < sources[datasetIndex].size()) { size += sources[datasetIndex][i]->size(); }
    }
    frame->reserve(size);
    for (size_t datasetIndex = 0; datasetIndex < datasetParam_.count(); datasetIndex++) {
      if (i < sources[datasetIndex].size()) {
        frame->insert(frame->end(), make_move_iterator(sources[datasetIndex][i]->begin()),
                      make_move_iterator(sources[datasetIndex][i]->end()));
      }
    }
    std::cout << "reader read "
              << "frameNumber=" << frame->header.seq << ", "
              << "points=" << frame->size() << std::endl;
  }
}

}  // namespace jpcc::io
