#include <sstream>

#include <io_tlv.h>

namespace jpcc::coder {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCDecoderTMC3<PointT>::JPCCDecoderTMC3(const JPCCDecoderParameter& parameter) :
    JPCCDecoder<PointT>(parameter), decoder(parameter.tmc3) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCDecoderTMC3<PointT>::decode(const std::vector<char>& encodedBytes, shared_ptr<void>& reconstructFrame) {
  reconstructFramePtr_ = &reconstructFrame;
  std::istringstream is(std::string(encodedBytes.begin(), encodedBytes.end()));

  pcc::PayloadBuffer buffer;
  while (true) {
    pcc::PayloadBuffer* bufferPtr = &buffer;
    readTlv(is, &buffer);

    // at end of file (or other error), flush decoder
    if (!is) { bufferPtr = nullptr; }

    if (decoder.decompress(bufferPtr, this)) {
      BOOST_THROW_EXCEPTION(std::logic_error("decompress point cloud error"));
    }

    if (!bufferPtr) { break; }
  }

  assert(*reconstructFramePtr_);

  reconstructFramePtr_ = nullptr;
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

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCDecoderTMC3<PointT>::onOutputCloud(const pcc::CloudFrame& frame) {
  if (reconstructFramePtr_) {
    auto reconstructFrame = make_shared<pcc::PCCPointSet3>(frame.cloud);
    for (size_t i = 0; i < frame.cloud.getPointCount(); i++) {  //
      (*reconstructFrame)[i] += frame.outputOrigin;
    }

    (*reconstructFramePtr_) = reconstructFrame;
  }
}

}  // namespace jpcc::coder