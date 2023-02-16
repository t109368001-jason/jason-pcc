#include <jpcc/decoder/JPCCDecoderTMC3.h>

#include <sstream>

#include <boost/log/trivial.hpp>

#include <io_tlv.h>
#include <PCCTMC3Decoder.h>

using namespace pcc_tmc3;

namespace jpcc::decoder {

class PCCTMC3Decoder3LambdaCallbacks : public PCCTMC3Decoder3::Callbacks {
 protected:
  const std::function<void(const CloudFrame& frame)>& onOutputCloud_;

 public:
  PCCTMC3Decoder3LambdaCallbacks(  // NOLINT(google-explicit-constructor)
      const std::function<void(const CloudFrame& frame)>& onOutputCloud);

 protected:
  void onOutputCloud(const CloudFrame& frame) override;
};

//////////////////////////////////////////////////////////////////////////////////////////////
PCCTMC3Decoder3LambdaCallbacks::PCCTMC3Decoder3LambdaCallbacks(
    const std::function<void(const CloudFrame& frame)>& onOutputCloud) :
    onOutputCloud_(onOutputCloud) {}

//////////////////////////////////////////////////////////////////////////////////////////////
void PCCTMC3Decoder3LambdaCallbacks::onOutputCloud(const CloudFrame& frame) { onOutputCloud_(frame); }

//////////////////////////////////////////////////////////////////////////////////////////////
bool JPCCDecoderTMC3::isConvertFromCoderTypeThreadSafe() { return true; }

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCDecoderTMC3::decode(std::istream& is, std::shared_ptr<void>& coderReconstructFrame) {
  std::function<void(const CloudFrame& frame)> onOutputCloud = [&](const CloudFrame& frame) {
    auto _frame           = std::make_shared<PCCPointSet3>(frame.cloud);
    coderReconstructFrame = _frame;
    for (size_t i = 0; i < _frame->getPointCount(); i++) {  //
      (*_frame)[i] += frame.outputOrigin;
    }
  };

  DecoderParams param       = {0};
  param.minGeomNodeSizeLog2 = 0;
  param.decodeMaxPoints     = 0;
  param.outputFpBits        = -1;
  PCCTMC3Decoder3LambdaCallbacks callback(onOutputCloud);
  PCCTMC3Decoder3                decoder(param);

  bool          hasSps        = false;
  auto          startPosition = is.tellg();
  PayloadBuffer buffer;
  while (true) {
    PayloadBuffer* bufferPtr = &buffer;
    auto           position  = is.tellg();
    readTlv(is, &buffer);

    if (!hasSps && bufferPtr->empty()) {
      coderReconstructFrame = make_shared<PCCPointSet3>();
      break;
    }

    if (buffer.type == PayloadType::kSequenceParameterSet) {
      if (hasSps) {
        bufferPtr = nullptr;
        is.seekg(position, std::ios::beg);
      }
      hasSps = true;
    }

    // at end of file (or other error), flush decoder
    if (!is) { bufferPtr = nullptr; }

    if (decoder.decompress(bufferPtr, &callback)) {
      BOOST_THROW_EXCEPTION(std::logic_error("decompress point cloud error"));
    }

    if (!bufferPtr) { break; }

    if (coderReconstructFrame) { break; }
  }
  if (coderReconstructFrame) {
    BOOST_LOG_TRIVIAL(info) << "bytes=" << is.tellg() - startPosition << ", "
                            << "points="
                            << std::static_pointer_cast<PCCPointSet3>(coderReconstructFrame)->getPointCount();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCDecoderTMC3::convertFromCoderType(const std::shared_ptr<void>& coderFrame, FramePtr& frame) {
  auto _coderFrame = std::static_pointer_cast<PCCPointSet3>(coderFrame);
  frame            = make_shared<Frame>();
  frame->resize(_coderFrame->getPointCount());
  for (size_t i = 0; i < _coderFrame->getPointCount(); i++) {  //
    auto point  = (*_coderFrame)[i];
    (*frame)[i] = PointType{point.x(), point.y(), point.z()};
  }
}

}  // namespace jpcc::decoder