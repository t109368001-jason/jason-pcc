#pragma once

#include <memory>
#include <vector>

#include <lvx_file.h>

#include <jpcc/common/Common.h>
#include <jpcc/io/DatasetParameter.h>
#include <jpcc/io/DatasetReaderParameter.h>
#include <jpcc/io/DatasetStreamReader.h>

namespace jpcc::io {

class LvxReader : public DatasetStreamReader {
 public:
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

  void load_(size_t datasetIndex, size_t startFrameNumber, size_t groupOfFramesSize) override;
};

}  // namespace jpcc::io
