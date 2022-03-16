#pragma once

#include <vector>

#include <jpcc/common/GroupOfFrame.h>
#include <jpcc/io/DatasetParameter.h>
#include <jpcc/io/DatasetReaderParameter.h>

namespace jpcc::io {

template <typename PointT = Point>
class DatasetReaderBase {
 public:
  using Ptr = shared_ptr<DatasetReaderBase>;

 protected:
  DatasetReaderParameter            param_;
  DatasetParameter                  datasetParam_;
  std::vector<int>                  datasetIndices_;
  std::vector<size_t>               currentFrameNumbers_;
  std::vector<GroupOfFrame<PointT>> frameBuffers_;

 public:
  DatasetReaderBase(DatasetReaderParameter param, DatasetParameter datasetParam);

  ~DatasetReaderBase();

  [[nodiscard]] DatasetReaderParameter getReaderParameter();

  [[nodiscard]] DatasetParameter getDatasetParameter();

  [[nodiscard]] bool isOpen();

  virtual void loadAll(size_t startFrameNumber, size_t groupOfFramesSize, GroupOfFrame<PointT>& frames);

  virtual void loadAll(size_t startFrameNumber, size_t groupOfFramesSize, GroupOfFrame<PointT>& frames, bool parallel);

  void close();

 protected:
  virtual void load(size_t                datasetIndex,
                    size_t                startFrameNumber,
                    size_t                groupOfFramesSize,
                    GroupOfFrame<PointT>& frames);

  virtual void open_(size_t datasetIndex, size_t startFrameNumber);

  [[nodiscard]] virtual bool isOpen_(size_t datasetIndex);

  [[nodiscard]] virtual bool isEof_(size_t datasetIndex);

  virtual void load_(size_t                datasetIndex,
                     size_t                startFrameNumber,
                     size_t                groupOfFramesSize,
                     GroupOfFrame<PointT>& frames);

  virtual void close_(size_t datasetIndex);
};

}  // namespace jpcc::io

#include <jpcc/io/impl/DatasetReaderBase.hpp>