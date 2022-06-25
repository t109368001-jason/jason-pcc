#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/io/DatasetReaderBase.h>
#include <jpcc/io/DatasetParameter.h>
#include <jpcc/io/DatasetReaderParameter.h>

namespace jpcc::io {

class DatasetReader : public DatasetReaderBase {
 public:
  using Ptr = shared_ptr<DatasetReader>;

 public:
  DatasetReader(DatasetReaderParameter param, DatasetParameter datasetParam);

  void loadAll(size_t startFrameNumber, size_t groupOfFramesSize, GroupOfFrame& frames);

  void loadAll(size_t startFrameNumber, size_t groupOfFramesSize, GroupOfFrame& frames, bool parallel);

  virtual void load(size_t datasetIndex, size_t startFrameNumber, size_t groupOfFramesSize, GroupOfFrame& frames) = 0;

 protected:
  void open_(size_t datasetIndex, size_t startFrameNumber) override = 0;

  [[nodiscard]] bool isOpen_(size_t datasetIndex) const override = 0;
};

}  // namespace jpcc::io
