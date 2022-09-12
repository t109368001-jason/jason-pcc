#include <sstream>

#include <io_tlv.h>

namespace jpcc::coder {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCEncoderTMC3<PointT>::JPCCEncoderTMC3(const JPCCEncoderParameter& parameter) :
    JPCCEncoder<PointT>(parameter), encoder_() {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCEncoderTMC3<PointT>::convertFromPCL(const FramePtr<PointT>& pclFrame, shared_ptr<void>& frame) {
  frame = make_shared<pcc::PCCPointSet3>();

  auto* frame_ = static_cast<pcc::PCCPointSet3*>(frame.get());

  frame_->resize(pclFrame->size());

  for (int i = 0; i < frame_->getPointCount(); i++) {
    (*frame_)[i] = pcc::PCCPointSet3::PointType(pclFrame->at(i).x, pclFrame->at(i).y, pclFrame->at(i).z);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCEncoderTMC3<PointT>::encode(const shared_ptr<void>& frame, std::vector<char>& encodedBytes) {
  encodedBytesPtr_ = &encodedBytes;
  encoder_.compress(*static_cast<pcc::PCCPointSet3*>(frame.get()),
                    static_cast<pcc::EncoderParams*>(&this->parameter_.tmc3), this, nullptr);
  encodedBytesPtr_ = nullptr;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCEncoderTMC3<PointT>::onOutputBuffer(const pcc::PayloadBuffer& buffer) {
  if (encodedBytesPtr_) {
    std::stringstream os;
    pcc::writeTlv(buffer, os);
#if !defined(NDEBUG)
    size_t oldSize = contextPtr_->encodedBytes.size();
#endif
    std::string tmpString = os.str();
    for (char& i : tmpString) { encodedBytesPtr_->push_back(i); }
    assert((buffer.size() + 5) == (contextPtr_->encodedBytes.size() - oldSize));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCEncoderTMC3<PointT>::onPostRecolour(const pcc::PCCPointSet3& set3) {}

}  // namespace jpcc::coder