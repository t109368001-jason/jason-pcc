#pragma once

#include <pcl/point_cloud.h>

#include <jpcc/common/Common.h>

namespace jpcc {

template <typename PointT = Point>
class GroupOfFrame : public std::vector<FramePtr<PointT>> {
 public:
  using Ptr = shared_ptr<GroupOfFrame>;
};

}  // namespace jpcc
