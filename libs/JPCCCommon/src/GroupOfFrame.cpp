#include <jpcc/common/GroupOfFrame.h>

namespace jpcc {
namespace common {

void GroupOfFrame::resize(const size_t size) { frames_.resize(size); };

size_t GroupOfFrame::size() { return frames_.size(); }

size_t GroupOfFrame::getStartFrameIndex() { return startFrameIndex_; }

void GroupOfFrame::setStartFrameIndex(const size_t startFrameIndex) { startFrameIndex_ = startFrameIndex_; }

std::vector<Frame>& GroupOfFrame::getFrames() { return frames_; }

Frame& Frame::operator[](size_t index) {
  assert(index < frames_.size());
  return frames_[index];
}

}  // namespace common
}  // namespace jpcc
