#pragma once

#include <vector>

#include <jpcc/common/GroupOfFrame.h>
#include <jpcc/io/DatasetParameter.h>
#include <jpcc/io/DatasetReaderParameter.h>

namespace jpcc::io {

class DatasetReaderBase {
 public:
  using Ptr = shared_ptr<DatasetReaderBase>;

 protected:
  DatasetReaderParameter               param_;
  DatasetParameter                     datasetParam_;
  std::vector<int>                     datasetIndices_;
  std::vector<size_t>                  currentFrameNumbers_;
  std::vector<std::vector<Frame::Ptr>> frameBuffers_;

 public:
  DatasetReaderBase(DatasetReaderParameter param, DatasetParameter datasetParam);

  ~DatasetReaderBase();

  [[nodiscard]] DatasetReaderParameter getReaderParameter();

  [[nodiscard]] bool isOpen();

  //  TODO change output std::vector<GroupOfFrame> to GroupOfFrame
  virtual void loadAll(size_t startFrameNumber, size_t groupOfFramesSize, std::vector<GroupOfFrame>& sources);

  //  TODO change output std::vector<GroupOfFrame> to GroupOfFrame
  virtual void loadAll(size_t                     startFrameNumber,
                       size_t                     groupOfFramesSize,
                       std::vector<GroupOfFrame>& sources,
                       bool                       parallel);

  virtual void load(size_t datasetIndex, size_t startFrameNumber, size_t groupOfFramesSize, GroupOfFrame& frames);

  void close();

 protected:
  virtual void open_(size_t datasetIndex, size_t startFrameNumber);

  [[nodiscard]] virtual bool isOpen_(size_t datasetIndex);

  [[nodiscard]] virtual bool isEof_(size_t datasetIndex);

  virtual void load_(size_t datasetIndex, size_t startFrameNumber, size_t groupOfFramesSize, GroupOfFrame& frames);

  virtual void close_(size_t datasetIndex);

  [[nodiscard]] const DatasetParameter& getDatasetParameter();
};

}  // namespace jpcc::io
