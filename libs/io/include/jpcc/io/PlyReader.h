#pragma once

#include <vector>

#include <jpcc/common/Common.h>
#include <jpcc/common/GroupOfFrame.h>
#include <jpcc/io/DatasetReaderBase.h>

namespace jpcc::io {

class PlyReader : public DatasetReaderBase {
 public:
  PlyReader(DatasetReaderParameter param, DatasetParameter datasetParam);

  void loadAll(size_t startFrameNumber, size_t groupOfFramesSize, GroupOfFrame& frames, bool parallel) override;
};

}  // namespace jpcc::io
