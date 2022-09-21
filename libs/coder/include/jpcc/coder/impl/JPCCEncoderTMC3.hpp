#include <sstream>

#include <io_tlv.h>

namespace jpcc::coder {

//////////////////////////////////////////////////////////////////////////////////////////////
PCCTMC3Encoder3LambdaCallbacks::PCCTMC3Encoder3LambdaCallbacks(
    const std::function<void(const pcc::PayloadBuffer& buffer)>& onOutputBuffer,
    const std::function<void(const pcc::PCCPointSet3& set3)>&    onPostRecolour) :
    onOutputBuffer_(onOutputBuffer), onPostRecolour_(onPostRecolour) {}

//////////////////////////////////////////////////////////////////////////////////////////////
void PCCTMC3Encoder3LambdaCallbacks::onOutputBuffer(const pcc::PayloadBuffer& buffer) { onOutputBuffer_(buffer); }

//////////////////////////////////////////////////////////////////////////////////////////////
void PCCTMC3Encoder3LambdaCallbacks::onPostRecolour(const pcc::PCCPointSet3& set3) { onPostRecolour_(set3); }

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCEncoderTMC3<PointT>::JPCCEncoderTMC3(const JPCCEncoderParameter& parameter) : JPCCEncoder<PointT>(parameter) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool JPCCEncoderTMC3<PointT>::isThreadSafe() {
  return true;
}

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
  pcc::EncoderParams   copiedParams = this->parameter_.tmc3;
  pcc::PCCTMC3Encoder3 encoder_;

  std::function<void(const pcc::PayloadBuffer& buffer)> onOutputBuffer =
      [&encodedBytes](const pcc::PayloadBuffer& buffer) {
        std::stringstream os;
        pcc::writeTlv(buffer, os);
#if !defined(NDEBUG)
        size_t oldSize = encodedBytes.size();
#endif
        std::string tmpString = os.str();
        for (char& i : tmpString) { encodedBytes.push_back(i); }
        assert((buffer.size() + 5) == (encodedBytes.size() - oldSize));
      };
  std::function<void(const pcc::PCCPointSet3& set3)> onPostRecolour = [](const pcc::PCCPointSet3& set3) {};

  PCCTMC3Encoder3LambdaCallbacks callback(onOutputBuffer, onPostRecolour);

  encoder_.compress(*static_cast<pcc::PCCPointSet3*>(frame.get()), static_cast<pcc::EncoderParams*>(&copiedParams),
                    &callback, nullptr);
}

}  // namespace jpcc::coder