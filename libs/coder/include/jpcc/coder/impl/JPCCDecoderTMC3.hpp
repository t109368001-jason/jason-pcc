#include <sstream>

#include <io_tlv.h>

namespace jpcc::coder {

//////////////////////////////////////////////////////////////////////////////////////////////
PCCTMC3Decoder3LambdaCallbacks::PCCTMC3Decoder3LambdaCallbacks(
    const std::function<void(const pcc::CloudFrame& frame)>& onOutputCloud) :
    onOutputCloud_(onOutputCloud) {}

//////////////////////////////////////////////////////////////////////////////////////////////
void PCCTMC3Decoder3LambdaCallbacks::onOutputCloud(const pcc::CloudFrame& frame) { onOutputCloud_(frame); }

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCDecoderTMC3<PointT>::JPCCDecoderTMC3(const JPCCDecoderParameter& parameter) : JPCCDecoder<PointT>(parameter) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCDecoderTMC3<PointT>::decode(const std::vector<char>& encodedBytes, shared_ptr<void>& reconstructFrame) {
  pcc::PCCTMC3Decoder3                              decoder(this->parameter_.tmc3);
  std::function<void(const pcc::CloudFrame& frame)> onOutputCloud = [&](const pcc::CloudFrame& frame) {
    reconstructFrame       = make_shared<pcc::PCCPointSet3>(frame.cloud);
    auto reconstructFrame_ = std::static_pointer_cast<pcc::PCCPointSet3>(reconstructFrame);
    for (size_t i = 0; i < frame.cloud.getPointCount(); i++) {  //
      (*reconstructFrame_)[i] += frame.outputOrigin;
    }
  };

  PCCTMC3Decoder3LambdaCallbacks callback(onOutputCloud);

  std::istringstream is(std::string(encodedBytes.begin(), encodedBytes.end()));

  pcc::PayloadBuffer buffer;
  while (true) {
    pcc::PayloadBuffer* bufferPtr = &buffer;
    readTlv(is, &buffer);

    // at end of file (or other error), flush decoder
    if (!is) { bufferPtr = nullptr; }

    if (decoder.decompress(bufferPtr, &callback)) {
      BOOST_THROW_EXCEPTION(std::logic_error("decompress point cloud error"));
    }

    if (!bufferPtr) { break; }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCDecoderTMC3<PointT>::convertToPCL(shared_ptr<void>& reconstructFrame, FramePtr<PointT>& reconstructPclFrame) {
  auto reconstructFrame_ = std::static_pointer_cast<pcc::PCCPointSet3>(reconstructFrame);

  reconstructPclFrame->resize(reconstructFrame_->getPointCount());

  for (int i = 0; i < reconstructPclFrame->size(); i++) {
    reconstructPclFrame->at(i) =
        PointT((*reconstructFrame_)[i].x(), (*reconstructFrame_)[i].y(), (*reconstructFrame_)[i].z());
  }
}

}  // namespace jpcc::coder