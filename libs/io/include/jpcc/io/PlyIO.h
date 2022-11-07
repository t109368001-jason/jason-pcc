#pragma once

#include <string>

#include <jpcc/common/Common.h>
#include <jpcc/io/DatasetParameter.h>

namespace jpcc::io {

void loadPly(GroupOfFrame&      frames,
             const std::string& filePath,
             size_t             startFrameNumber,
             size_t             endFrameNumber,
             bool               parallel = false);

void savePly(const GroupOfFrame& frames, const std::string& filePath, const bool parallel);

}  // namespace jpcc::io
