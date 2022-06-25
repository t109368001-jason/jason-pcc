#pragma once

#include <jpcc/common/Common.h>

namespace jpcc::process {

void split(const FramePtr& input, const IndicesPtr& indices, const FramePtr& output, const FramePtr& outputNegative);

void quantize(const FramePtr& frame, double resolution);

void quantize(const GroupOfFrame& frames, double resolution, bool parallel);

}  // namespace jpcc::process
