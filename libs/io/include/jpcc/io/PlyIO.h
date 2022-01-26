#pragma once

#include <jpcc/common/GroupOfFrame.h>
#include <jpcc/io/DatasetParameter.h>

namespace jpcc::io {

void load(GroupOfFrame& groupOfFrame,
          std::string   filePath,
          size_t        startFrameNumber,
          size_t        endFrameNumber,
          bool          parallel = false);

void save(const GroupOfFrame& groupOfFrame, std::string filePath, bool parallel = false);

}  // namespace jpcc::io
