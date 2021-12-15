#include <jpcc/io/DatasetReader.h>

#include <exception>
#include <execution>

namespace jpcc {
namespace io {

using namespace std;
using namespace jpcc::common;

DatasetReader::DatasetReader(const DatasetParameter& datasetParam) : datasetParam_(datasetParam) {
  assert(datasetParam_.totalFiles > 0);
}

void DatasetReader::loadAll(vector<GroupOfFrame>& sources,
                            const size_t          startFrameIndex,
                            const size_t          groupOfFramesSize,
                            const bool            parallel) {
  assert(groupOfFramesSize > 0);
  vector<int> datasetIndices(datasetParam_.totalFiles);
  generate(datasetIndices.begin(), datasetIndices.end(), [n = 0]() mutable { return n++; });

  sources.resize(datasetIndices.size());
  if (parallel) {
    for_each(execution::par, datasetIndices.begin(), datasetIndices.end(), [&](size_t datasetIndex) {
      load(datasetIndex, sources[datasetIndex], startFrameIndex, groupOfFramesSize, parallel);
    });
  } else {
    for_each(datasetIndices.begin(), datasetIndices.end(), [&](size_t datasetIndex) {
      load(datasetIndex, sources[datasetIndex], startFrameIndex, groupOfFramesSize, parallel);
    });
  }
}

void DatasetReader::load(const size_t  datasetIndex,
                         GroupOfFrame& frames,
                         const size_t  startFrameIndex,
                         const size_t  groupOfFramesSize,
                         const bool    parallel) {
  throw logic_error(string("Not Implemented ") + BOOST_CURRENT_FUNCTION);
}

const DatasetParameter& DatasetReader::getDatasetParameter() { return datasetParam_; }

}  // namespace io
}  // namespace jpcc
