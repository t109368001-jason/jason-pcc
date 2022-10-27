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
bool JPCCEncoderTMC3<PointT>::isConvertFromPCLThreadSafe() {
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool JPCCEncoderTMC3<PointT>::isEncodeThreadSafe() {
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCEncoderTMC3<PointT>::convertFromPCL(const FramePtr<PointT>& pclFrame, shared_ptr<void>& frame) {
  frame       = make_shared<pcc::PCCPointSet3>();
  auto _frame = std::static_pointer_cast<pcc::PCCPointSet3>(frame);

  _frame->resize(pclFrame->size());

  for (int i = 0; i < _frame->getPointCount(); i++) {
    (*_frame)[i] = pcc::PCCPointSet3::PointType((*pclFrame)[i].x, (*pclFrame)[i].y, (*pclFrame)[i].z);
  }
  std::cout << __FUNCTION__ << "() "
            << "pclPoints=" << pclFrame->size() << ", "
            << "tmc3Points=" << _frame->getPointCount() << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCEncoderTMC3<PointT>::encode(const shared_ptr<void>& frame, std::vector<char>& encodedBytes) {
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

  auto _frame = std::static_pointer_cast<pcc::PCCPointSet3>(frame);
  if (_frame->getPointCount() == 0) {
    pcc::PayloadBuffer buffer;
    buffer.type = pcc::PayloadType::kSequenceParameterSet;

    onOutputBuffer(buffer);
    return;
  }

  pcc::EncoderParams             param = this->parameter_.tmc3;
  PCCTMC3Encoder3LambdaCallbacks callback(onOutputBuffer, onPostRecolour);
  pcc::PCCTMC3Encoder3           encoder;
  encoder.compress(*_frame, &param, &callback, nullptr);
  std::cout << __FUNCTION__ << "() "
            << "bytes=" << encodedBytes.size() << std::endl;
}

}  // namespace jpcc::coder