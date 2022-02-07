#pragma once

#include <jpcc/common/GroupOfFrame.h>
#include <jpcc/io/DatasetParameter.h>

namespace jpcc::io {

void loadPly(
    GroupOfFrame& frames, std::string filePath, size_t startFrameNumber, size_t endFrameNumber, bool parallel = false);

void savePly(const GroupOfFrame& frames, std::string filePath, bool parallel = false);

}  // namespace jpcc::io
