#pragma once

#include <vector>

#include <pcl/point_cloud.h>

#include <jpcc/common/Common.h>

namespace jpcc {

template <typename PointT = Point>
class GroupOfFrame : public std::vector<FramePtr<PointT>> {
 public:
  using std::vector<FramePtr<PointT>>::vector;
  using Ptr = shared_ptr<GroupOfFrame>;
};

}  // namespace jpcc
