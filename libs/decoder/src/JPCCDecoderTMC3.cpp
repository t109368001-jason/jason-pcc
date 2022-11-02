#include <jpcc/decoder/JPCCDecoderTMC3.h>

#include <sstream>

#include "io_tlv.h"

namespace jpcc::decoder {

//////////////////////////////////////////////////////////////////////////////////////////////
PCCTMC3Decoder3LambdaCallbacks::PCCTMC3Decoder3LambdaCallbacks(
    const std::function<void(const pcc::CloudFrame& frame)>& onOutputCloud) :
    onOutputCloud_(onOutputCloud) {}

//////////////////////////////////////////////////////////////////////////////////////////////
void PCCTMC3Decoder3LambdaCallbacks::onOutputCloud(const pcc::CloudFrame& frame) { onOutputCloud_(frame); }

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCDecoderTMC3::decode(std::istream& is, FramePtr& reconstructFrame) {
  std::function<void(const pcc::CloudFrame& frame)> onOutputCloud = [&](const pcc::CloudFrame& frame) {
    reconstructFrame = make_shared<Frame>(frame.cloud);
    for (size_t i = 0; i < frame.cloud.getPointCount(); i++) {  //
      (*reconstructFrame)[i] += frame.outputOrigin;
    }
  };

  pcc::DecoderParams param  = {0};
  param.minGeomNodeSizeLog2 = 0;
  param.decodeMaxPoints     = 0;
  param.outputFpBits        = -1;
  PCCTMC3Decoder3LambdaCallbacks callback(onOutputCloud);
  pcc::PCCTMC3Decoder3           decoder(param);

  bool               hasSps        = false;
  auto               startPosition = is.tellg();
  pcc::PayloadBuffer buffer;
  while (true) {
    pcc::PayloadBuffer* bufferPtr = &buffer;
    auto                position  = is.tellg();
    readTlv(is, &buffer);

    if (!hasSps && bufferPtr->empty()) {
      reconstructFrame = make_shared<Frame>();
      break;
    }

    if (buffer.type == pcc::PayloadType::kSequenceParameterSet) {
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

    if (reconstructFrame) { break; }
  }
  if (reconstructFrame) {
    std::cout << __FUNCTION__ << "() "
              << "bytes=" << is.tellg() - startPosition << ", "
              << "points=" << std::static_pointer_cast<pcc::PCCPointSet3>(reconstructFrame)->getPointCount()
              << std::endl;
  }
}

}  // namespace jpcc::decoder