#include <jpcc/common/GroupOfFrame.h>

namespace jpcc {

Frame::Ptr& GroupOfFrame::at(size_t index) { return frames_.at(index); }

const Frame::Ptr& GroupOfFrame::at(size_t index) const { return frames_.at(index); }

void GroupOfFrame::resize(const size_t size) { frames_.resize(size); }

void GroupOfFrame::clear() { frames_.clear(); }

size_t GroupOfFrame::size() const { return frames_.size(); }

std::vector<Frame::Ptr>& GroupOfFrame::getFrames() { return frames_; }

const std::vector<Frame::Ptr>& GroupOfFrame::getFrames() const { return frames_; }

}  // namespace jpcc
