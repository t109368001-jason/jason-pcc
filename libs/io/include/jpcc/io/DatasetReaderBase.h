#pragma once

#include <vector>

#include <jpcc/common/GroupOfFrame.h>
#include <jpcc/io/DatasetParameter.h>
#include <jpcc/io/DatasetReaderParameter.h>

namespace jpcc::io {

using Frame        = common::Frame;
using GroupOfFrame = common::GroupOfFrame;

class DatasetReaderBase {
 public:
  using Ptr = shared_ptr<DatasetReaderBase>;

 protected:
  DatasetReaderParameter               param_;
  DatasetParameter                     datasetParam_;
  std::vector<int>                     datasetIndices_;
  std::vector<size_t>                  currentFrameIndices_;
  std::vector<std::vector<Frame::Ptr>> frameBuffers_;

 public:
  DatasetReaderBase(DatasetReaderParameter param, DatasetParameter datasetParam);

  ~DatasetReaderBase();

  [[nodiscard]] DatasetReaderParameter getReaderParameter();

  [[nodiscard]] bool isOpen();

  void loadAll(size_t                     startFrameIndex,
               size_t                     groupOfFramesSize,
               std::vector<GroupOfFrame>& sources,
               bool                       parallel = false);

  void load(size_t datasetIndex, size_t startFrameIndex, size_t groupOfFramesSize, GroupOfFrame& frames);

  void close();

 protected:
  virtual void open_(size_t datasetIndex, size_t startFrameIndex);

  [[nodiscard]] virtual bool isOpen_(size_t datasetIndex);

  [[nodiscard]] virtual bool isEof_(size_t datasetIndex);

  virtual void load_(size_t datasetIndex, size_t startFrameIndex, size_t groupOfFramesSize, GroupOfFrame& frames);

  virtual void close_(size_t datasetIndex);

  [[nodiscard]] const DatasetParameter& getDatasetParameter();
};

}  // namespace jpcc::io
