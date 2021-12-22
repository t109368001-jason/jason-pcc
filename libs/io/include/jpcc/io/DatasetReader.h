#ifndef JPCC_IO_DATASET_READER_H_
#define JPCC_IO_DATASET_READER_H_

#include <vector>

#include <jpcc/common/GroupOfFrame.h>
#include <jpcc/io/DatasetParameter.h>

namespace jpcc {
namespace io {

using GroupOfFrame = common::GroupOfFrame;

class DatasetReader {
 protected:
  const DatasetParameter& datasetParam_;

 public:
  DatasetReader(const DatasetParameter& datasetParam);

  virtual void loadAll(std::vector<GroupOfFrame>& sources,
                       const size_t               startFrameIndex,
                       const size_t               groupOfFramesSize,
                       const bool                 parallel = false);

  virtual void load(const size_t  datasetIndex,
                    GroupOfFrame& frames,
                    const size_t  startFrameIndex,
                    const size_t  groupOfFramesSize,
                    const bool    parallel = false);

  const DatasetParameter& getDatasetParameter();
};

}  // namespace io
}  // namespace jpcc

#endif  // JPCC_IO_DATASET_READER_H_