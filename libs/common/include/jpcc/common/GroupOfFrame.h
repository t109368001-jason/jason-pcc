#ifndef JPCC_COMMON_GROUP_OF_CLOUD_H_
#define JPCC_COMMON_GROUP_OF_CLOUD_H_

#include <jpcc/common/Frame.h>

namespace jpcc::common {

class GroupOfFrame {
 public:
  using Ptr = shared_ptr<GroupOfFrame>;

 protected:
  size_t                  startFrameIndex_;
  std::vector<Frame::Ptr> frames_;

 public:
  [[nodiscard]] Frame::Ptr& at(size_t index);

  void resize(size_t size);

  [[nodiscard]] size_t size() const;

  [[nodiscard]] size_t getStartFrameIndex() const;

  void setStartFrameIndex(size_t startFrameIndex);

  [[nodiscard]] std::vector<Frame::Ptr>& getFrames();
};

}  // namespace jpcc::common

#endif  // JPCC_COMMON_GROUP_OF_CLOUD_H_