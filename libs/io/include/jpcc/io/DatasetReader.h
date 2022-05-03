#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/common/GroupOfFrame.h>
#include <jpcc/io/DatasetParameter.h>
#include <jpcc/io/DatasetReaderParameter.h>

namespace jpcc::io {

template <typename PointT = Point>
class DatasetReader {
 public:
  using Ptr          = shared_ptr<DatasetReader>;
  using Frame        = jpcc::Frame<PointT>;
  using FramePtr     = typename Frame::Ptr;
  using GroupOfFrame = jpcc::GroupOfFrame<PointT>;

 protected:
  DatasetReaderParameter param_;
  DatasetParameter       datasetParam_;
  std::vector<int>       datasetIndices_;
  std::vector<size_t>    currentFrameNumbers_;

 public:
  DatasetReader(DatasetReaderParameter param, DatasetParameter datasetParam);

  ~DatasetReader();

  [[nodiscard]] const DatasetReaderParameter& getReaderParameter() const;

  [[nodiscard]] const DatasetParameter& getDatasetParameter() const;

  void loadAll(size_t startFrameNumber, size_t groupOfFramesSize, GroupOfFrame& frames);

  void loadAll(size_t startFrameNumber, size_t groupOfFramesSize, GroupOfFrame& frames, bool parallel);

  void open(size_t startFrameNumber);

  [[nodiscard]] bool isOpen() const;

  void close();

  virtual void load(size_t datasetIndex, size_t startFrameNumber, size_t groupOfFramesSize, GroupOfFrame& frames) = 0;

 protected:
  virtual void open_(size_t datasetIndex, size_t startFrameNumber) = 0;

  [[nodiscard]] virtual bool isOpen_(size_t datasetIndex) const = 0;

  virtual void close_(size_t datasetIndex) = 0;
};

}  // namespace jpcc::io

#include <jpcc/io/impl/DatasetReader.hpp>
