#pragma once

#include <vector>

#include <jpcc/common/Common.h>
#include <jpcc/io/DatasetParameter.h>
#include <jpcc/io/DatasetReader.h>
#include <jpcc/io/DatasetReaderParameter.h>

namespace jpcc::io {

class DatasetStreamReader : public DatasetReader {
 public:
  using Ptr = shared_ptr<DatasetStreamReader>;

 protected:
  size_t                            capacity_;
  std::vector<bool>                 eof_;
  std::vector<GroupOfFrame>         frameBuffers_;
  std::vector<std::vector<bool>>    finishVectors_;
  std::vector<std::vector<int64_t>> timestampVectors_;

 public:
  DatasetStreamReader(DatasetReaderParameter param, DatasetParameter datasetParam);

  void load(size_t        datasetIndex,
            size_t        startFrameNumber,
            size_t        groupOfFramesSize,
            GroupOfFrame& frames,
            bool          parallel) override;

 protected:
  void open_(size_t datasetIndex, size_t startFrameNumber) override = 0;

  [[nodiscard]] bool isOpen_(size_t datasetIndex) const override = 0;

  [[nodiscard]] virtual bool isEof_(size_t datasetIndex) const;

  virtual void load_(size_t datasetIndex, size_t startFrameNumber, size_t groupOfFramesSize) = 0;
};

}  // namespace jpcc::io
