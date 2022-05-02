#pragma

#include <jpcc/common/Common.h>

namespace jpcc::process {

template <typename PointT = Point>
void split(const FramePtr<PointT>& input,
           const IndicesPtr&       indices,
           const FramePtr<PointT>& output,
           const FramePtr<PointT>& outputNegative);

template <typename PointT = Point>
void quantize(const FramePtr<PointT>& frame, double resolution);

template <typename PointT = Point>
void quantize(const GroupOfFrame<PointT>& frames, double resolution, bool parallel);

}  // namespace jpcc::process

#include <jpcc/process/impl/Process.hpp>