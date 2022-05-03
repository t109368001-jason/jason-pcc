#pragma once

#include <vector>

#include <jpcc/io/DatasetReader.h>

namespace jpcc::io {

template <typename PointT = Point>
class DatasetStreamReader : public DatasetReader<PointT> {
 public:
  using Ptr          = shared_ptr<DatasetStreamReader>;
  using Frame        = jpcc::Frame<PointT>;
  using FramePtr     = typename Frame::Ptr;
  using GroupOfFrame = jpcc::GroupOfFrame<PointT>;

 protected:
  std::vector<GroupOfFrame> frameBuffers_;

 public:
  DatasetStreamReader(DatasetReaderParameter param, DatasetParameter datasetParam);

  void load(size_t datasetIndex, size_t startFrameNumber, size_t groupOfFramesSize, GroupOfFrame& frames) override;

 protected:
  void open_(size_t datasetIndex, size_t startFrameNumber) override = 0;

  [[nodiscard]] bool isOpen_(size_t datasetIndex) const override = 0;

  void close_(size_t datasetIndex) override;

  [[nodiscard]] virtual bool isEof_(size_t datasetIndex) const;

  virtual void load_(size_t datasetIndex, size_t startFrameNumber, size_t groupOfFramesSize, GroupOfFrame& frames) = 0;
};

}  // namespace jpcc::io

#include <jpcc/io/impl/DatasetStreamReader.hpp>