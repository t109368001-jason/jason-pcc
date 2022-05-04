#pragma once

#include <vector>

#include <lvx_file.h>

#include <jpcc/common/Common.h>
#include <jpcc/common/GroupOfFrame.h>
#include <jpcc/io/DatasetStreamReader.h>

namespace jpcc::io {

template <typename PointT = Point>
class LvxReader : public DatasetStreamReader<PointT> {
 public:
  using Frame         = jpcc::Frame<PointT>;
  using FramePtr      = typename Frame::Ptr;
  using GroupOfFrame  = jpcc::GroupOfFrame<PointT>;
  using LvxHandler    = livox_ros::LvxFileHandle;
  using LvxHandlerPtr = std::unique_ptr<LvxHandler, void (*)(LvxHandler*)>;

 protected:
  size_t                     capacity_;
  std::vector<LvxHandlerPtr> lvxs_;

 public:
  LvxReader(DatasetReaderParameter param, DatasetParameter datasetParam);

 protected:
  void open_(size_t datasetIndex, size_t startFrameNumber) override;

  [[nodiscard]] bool isOpen_(size_t datasetIndex) const override;

  [[nodiscard]] bool isEof_(size_t datasetIndex) const override;

  void load_(size_t datasetIndex, size_t startFrameNumber, size_t groupOfFramesSize, GroupOfFrame& frames) override;
};

}  // namespace jpcc::io

#include <jpcc/io/impl/LvxReader.hpp>
