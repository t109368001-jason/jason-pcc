#pragma

#include <jpcc/common/Common.h>

namespace jpcc::process {

template <typename PointT>
void split(const FramePtr<PointT>& input,
           const IndicesPtr&       indices,
           const FramePtr<PointT>& output,
           const FramePtr<PointT>& outputNegative);

}

#include <jpcc/process/impl/Process.hpp>