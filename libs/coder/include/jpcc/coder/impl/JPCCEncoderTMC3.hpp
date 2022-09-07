#include <sstream>

#include <io_tlv.h>

namespace jpcc::coder {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCEncoderTMC3<PointT>::JPCCEncoderTMC3(const JPCCEncoderParameter& parameter) :
    JPCCEncoder<PointT>(parameter), encoder_() {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCEncoderTMC3<PointT>::convertFromPCL(JPCCCoderContext<PointT>& context) {
  context.frame = make_shared<pcc::PCCPointSet3>();

  auto* frame = static_cast<pcc::PCCPointSet3*>(context.frame.get());

  frame->resize(context.pclFrame->size());

  for (int i = 0; i < frame->getPointCount(); i++) {
    (*frame)[i] =
        pcc::PCCPointSet3::PointType(context.pclFrame->at(i).x, context.pclFrame->at(i).y, context.pclFrame->at(i).z);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCEncoderTMC3<PointT>::encode(JPCCCoderContext<PointT>& context) {
  contextPtr_ = &context;
  encoder_.compress(*static_cast<pcc::PCCPointSet3*>(context.frame.get()),
                    static_cast<pcc::EncoderParams*>(&this->parameter_.tmc3), this, nullptr);
  contextPtr_ = nullptr;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCEncoderTMC3<PointT>::onOutputBuffer(const pcc::PayloadBuffer& buffer) {
  if (contextPtr_) {
    std::stringstream os;
    pcc::writeTlv(buffer, os);
#if !defined(NDEBUG)
    size_t oldSize = contextPtr_->encodedBytes.size();
#endif
    std::string tmpString = os.str();
    for (char& i : tmpString) { contextPtr_->encodedBytes.push_back(i); }
    assert((buffer.size() + 5) == (contextPtr_->encodedBytes.size() - oldSize));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCEncoderTMC3<PointT>::onPostRecolour(const pcc::PCCPointSet3& set3) {}

}  // namespace jpcc::coder