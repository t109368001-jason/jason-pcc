#pragma once

#include <jpcc/common/GroupOfFrame.h>
#include <jpcc/io/DatasetParameter.h>

namespace jpcc::io {

void loadPly(GroupOfFrame& groupOfFrame,
             std::string   filePath,
             size_t        startFrameNumber,
             size_t        endFrameNumber,
             bool          parallel = false);

void savePly(const GroupOfFrame& groupOfFrame, std::string filePath, bool parallel = false);

}  // namespace jpcc::io
