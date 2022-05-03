#pragma once

#include <vector>

#include <jpcc/common/Common.h>
#include <jpcc/common/GroupOfFrame.h>
#include <jpcc/io/DatasetStreamReader.h>

namespace jpcc::io {

template <typename PointT = Point>
class PlyReader : public DatasetStreamReader<PointT> {
 public:
  using GroupOfFrame = jpcc::GroupOfFrame<PointT>;

 public:
  PlyReader(DatasetReaderParameter param, DatasetParameter datasetParam);

  void loadAll(size_t startFrameNumber, size_t groupOfFramesSize, GroupOfFrame& frames, bool parallel) override;
};

}  // namespace jpcc::io

#include <jpcc/io/impl/PlyReader.hpp>
