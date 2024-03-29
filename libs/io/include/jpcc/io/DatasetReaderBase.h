#pragma once

#include <vector>

#include <jpcc/common/Common.h>
#include <jpcc/io/DatasetParameter.h>
#include <jpcc/io/DatasetReaderParameter.h>

namespace jpcc::io {

class DatasetReaderBase {
 public:
  using Ptr = shared_ptr<DatasetReaderBase>;

 protected:
  DatasetReaderParameter param_;
  DatasetParameter       datasetParam_;
  std::vector<int>       datasetIndices_;
  std::vector<size_t>    currentFrameNumbers_;

 public:
  DatasetReaderBase(DatasetReaderParameter param, DatasetParameter datasetParam);

  void open(size_t startFrameNumber);

  [[nodiscard]] bool isOpen() const;

 protected:
  virtual void open_(size_t datasetIndex, size_t startFrameNumber) = 0;

  [[nodiscard]] virtual bool isOpen_(size_t datasetIndex) const = 0;
};

}  // namespace jpcc::io
