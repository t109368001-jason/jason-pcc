#pragma once

#include <vector>

#include <lvx_file.h>

#include <jpcc/common/Common.h>
#include <jpcc/common/GroupOfFrame.h>
#include <jpcc/io/DatasetReaderBase.h>

namespace jpcc::io {

class LvxReader : public DatasetReaderBase {
 protected:
  size_t                                            capacity_;
  std::vector<shared_ptr<livox_ros::LvxFileHandle>> lvxs_;

 public:
  LvxReader(DatasetReaderParameter param, DatasetParameter datasetParam);

 protected:
  void open_(size_t datasetIndex, size_t startFrameNumber) override;

  [[nodiscard]] bool isOpen_(size_t datasetIndex) override;

  [[nodiscard]] bool isEof_(size_t datasetIndex) override;

  void load_(size_t datasetIndex, size_t startFrameNumber, size_t groupOfFramesSize, GroupOfFrame& frames) override;

  void close_(size_t datasetIndex) override;
};

}  // namespace jpcc::io
