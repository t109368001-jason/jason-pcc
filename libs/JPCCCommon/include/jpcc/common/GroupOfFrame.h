#ifndef JPCC_COMMON_GROUP_OF_CLOUD_H_
#define JPCC_COMMON_GROUP_OF_CLOUD_H_

#include <jpcc/common/Frame.h>

namespace jpcc {
namespace common {

class GroupOfFrame {
 protected:
  size_t             startFrameIndex_;
  std::vector<Frame> frames_;

 public:
  void resize(const size_t size);

  size_t size();

  size_t getStartFrameIndex();

  void setStartFrameIndex(const size_t startFrameIndex);

  std::vector<Frame>& getFrames();

  Frame& operator[](size_t index);
};

}  // namespace common
}  // namespace jpcc

#endif  // JPCC_COMMON_GROUP_OF_CLOUD_H_