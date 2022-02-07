#pragma once

#include <pcl/point_cloud.h>

#include <jpcc/common/Common.h>

namespace jpcc {

class GroupOfFrame : public std::vector<Frame::Ptr> {
 public:
  using Ptr = shared_ptr<GroupOfFrame>;
};

}  // namespace jpcc
