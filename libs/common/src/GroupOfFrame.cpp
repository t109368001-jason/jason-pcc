#include <jpcc/common/GroupOfFrame.h>

namespace jpcc {

Frame::Ptr& GroupOfFrame::at(size_t index) { return frames_.at(index); }

void GroupOfFrame::resize(const size_t size) { frames_.resize(size); }

size_t GroupOfFrame::size() const { return frames_.size(); }

size_t GroupOfFrame::getStartFrameNumber() const { return startFrameNumber_; }

void GroupOfFrame::setStartFrameNumber(const size_t startFrameNumber) { startFrameNumber_ = startFrameNumber; }

std::vector<Frame::Ptr>& GroupOfFrame::getFrames() { return frames_; }

}  // namespace jpcc
