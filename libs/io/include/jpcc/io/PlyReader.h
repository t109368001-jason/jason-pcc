#pragma once

#include <vector>

#include <jpcc/common/Common.h>
#include <jpcc/io/DatasetParameter.h>
#include <jpcc/io/DatasetReader.h>
#include <jpcc/io/DatasetReaderParameter.h>

namespace jpcc::io {

class PlyReader : public DatasetReader {
 public:
  PlyReader(DatasetReaderParameter param, DatasetParameter datasetParam);

  void load(size_t datasetIndex, size_t startFrameNumber, size_t groupOfFramesSize, GroupOfFrame& frames) override;

  void load(
      size_t datasetIndex, size_t startFrameNumber, size_t groupOfFramesSize, GroupOfFrame& frames, bool parallel);

 protected:
  void open_(size_t datasetIndex, size_t startFrameNumber) override;

  [[nodiscard]] bool isOpen_(size_t datasetIndex) const override;
};

}  // namespace jpcc::io
