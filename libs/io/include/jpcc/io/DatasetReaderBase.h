#pragma once

#include <vector>

#include <jpcc/common/GroupOfFrame.h>
#include <jpcc/io/DatasetParameter.h>
#include <jpcc/io/DatasetReaderParameter.h>

namespace jpcc::io {

template <typename PointT = Point>
class DatasetReaderBase {
 public:
  using Ptr          = shared_ptr<DatasetReaderBase>;
  using Frame        = jpcc::Frame<PointT>;
  using FramePtr     = typename Frame::Ptr;
  using GroupOfFrame = jpcc::GroupOfFrame<PointT>;

 protected:
  DatasetReaderParameter    param_;
  DatasetParameter          datasetParam_;
  std::vector<int>          datasetIndices_;
  std::vector<size_t>       currentFrameNumbers_;
  std::vector<GroupOfFrame> frameBuffers_;

 public:
  DatasetReaderBase(DatasetReaderParameter param, DatasetParameter datasetParam);

  ~DatasetReaderBase();

  [[nodiscard]] const DatasetReaderParameter& getReaderParameter() const;

  [[nodiscard]] const DatasetParameter& getDatasetParameter() const;

  [[nodiscard]] bool isOpen() const;

  virtual void loadAll(size_t startFrameNumber, size_t groupOfFramesSize, GroupOfFrame& frames);

  virtual void loadAll(size_t startFrameNumber, size_t groupOfFramesSize, GroupOfFrame& frames, bool parallel);

  void close();

  // protected:
  virtual void load(size_t datasetIndex, size_t startFrameNumber, size_t groupOfFramesSize, GroupOfFrame& frames);

  virtual void open_(size_t datasetIndex, size_t startFrameNumber);

  [[nodiscard]] virtual bool isOpen_(size_t datasetIndex) const;

  [[nodiscard]] virtual bool isEof_(size_t datasetIndex) const;

  virtual void load_(size_t datasetIndex, size_t startFrameNumber, size_t groupOfFramesSize, GroupOfFrame& frames);

  virtual void close_(size_t datasetIndex);
};

}  // namespace jpcc::io

#include <jpcc/io/impl/DatasetReaderBase.hpp>