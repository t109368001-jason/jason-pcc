#pragma once

#include <pcl/point_cloud.h>

#include <jpcc/common/Common.h>

namespace jpcc {

class GroupOfFrame {
 public:
  using Ptr = shared_ptr<GroupOfFrame>;

 protected:
  std::vector<Frame::Ptr> frames_;

 public:
  [[nodiscard]] Frame::Ptr& at(size_t index);

  [[nodiscard]] const Frame::Ptr& at(size_t index) const;

  void resize(size_t size);

  void clear();

  [[nodiscard]] size_t size() const;

  [[nodiscard]] std::vector<Frame::Ptr>& getFrames();

  [[nodiscard]] const std::vector<Frame::Ptr>& getFrames() const;
};

}  // namespace jpcc
