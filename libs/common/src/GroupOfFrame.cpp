#include <jpcc/common/GroupOfFrame.h>

namespace jpcc::common {

Frame::Ptr& GroupOfFrame::at(size_t index) { return frames_.at(index); }

void GroupOfFrame::resize(const size_t size) { frames_.resize(size); }

size_t GroupOfFrame::size() const { return frames_.size(); }

size_t GroupOfFrame::getStartFrameIndex() const { return startFrameIndex_; }

void GroupOfFrame::setStartFrameIndex(const size_t startFrameIndex) { startFrameIndex_ = startFrameIndex; }

std::vector<Frame::Ptr>& GroupOfFrame::getFrames() { return frames_; }

}  // namespace jpcc::common
