#include <jpcc/common/GroupOfFrame.h>

namespace jpcc {
namespace common {

Frame::Ptr& GroupOfFrame::at(size_t index) { return frames_.at(index); }

void GroupOfFrame::resize(const size_t size) { frames_.resize(size); };

size_t GroupOfFrame::size() { return frames_.size(); }

size_t GroupOfFrame::getStartFrameIndex() { return startFrameIndex_; }

void GroupOfFrame::setStartFrameIndex(const size_t startFrameIndex) { startFrameIndex_ = startFrameIndex_; }

std::vector<Frame::Ptr>& GroupOfFrame::getFrames() { return frames_; }

}  // namespace common
}  // namespace jpcc
