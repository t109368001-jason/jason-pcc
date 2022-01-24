#pragma once

#include <jpcc/common/Frame.h>

namespace jpcc::common {

class GroupOfFrame {
 public:
  using Ptr = shared_ptr<GroupOfFrame>;

 protected:
  size_t                  startFrameNumber_;
  std::vector<Frame::Ptr> frames_;

 public:
  [[nodiscard]] Frame::Ptr& at(size_t index);

  void resize(size_t size);

  [[nodiscard]] size_t size() const;

  [[nodiscard]] size_t getStartFrameNumber() const;

  void setStartFrameNumber(const size_t startFrameNumber);

  [[nodiscard]] std::vector<Frame::Ptr>& getFrames();
};

}  // namespace jpcc::common
