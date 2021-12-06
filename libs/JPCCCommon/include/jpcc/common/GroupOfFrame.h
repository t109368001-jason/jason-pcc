#ifndef JPCC_COMMON_GROUP_OF_CLOUD_H_
#define JPCC_COMMON_GROUP_OF_CLOUD_H_

#include <jpcc/common/Frame.h>

namespace jpcc {
namespace common {

class GroupOfFrame {
 protected:
  size_t                  startFrameIndex_;
  std::vector<Frame::Ptr> frames_;

 public:
  Frame::Ptr& at(size_t index);

  void resize(const size_t size);

  size_t size();

  size_t getStartFrameIndex();

  void setStartFrameIndex(const size_t startFrameIndex);

  std::vector<Frame::Ptr>& getFrames();
};

}  // namespace common
}  // namespace jpcc

#endif  // JPCC_COMMON_GROUP_OF_CLOUD_H_