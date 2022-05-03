#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/common/GroupOfFrame.h>
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

  ~DatasetReaderBase();

  [[nodiscard]] const DatasetReaderParameter& getReaderParameter() const;

  [[nodiscard]] const DatasetParameter& getDatasetParameter() const;

  void open(size_t startFrameNumber);

  [[nodiscard]] bool isOpen() const;

  void close();

 protected:
  virtual void open_(size_t datasetIndex, size_t startFrameNumber) = 0;

  [[nodiscard]] virtual bool isOpen_(size_t datasetIndex) const = 0;

  virtual void close_(size_t datasetIndex) = 0;
};

}  // namespace jpcc::io
