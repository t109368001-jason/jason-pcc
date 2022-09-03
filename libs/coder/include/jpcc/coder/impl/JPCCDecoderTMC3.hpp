#include <strstream>

#include <io_tlv.h>

namespace jpcc::coder {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCDecoderTMC3<PointT>::JPCCDecoderTMC3(const JPCCDecoderParameter& parameter) :
    JPCCDecoder<PointT>(parameter), decoder(parameter.tmc3) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCDecoderTMC3<PointT>::decode(JPCCCoderContext<PointT>& context) {
  contextPtr_ = &context;
  contextPtr_->encodedBytes;
  std::istrstream is(contextPtr_->encodedBytes.data(), contextPtr_->encodedBytes.size());

  pcc::PayloadBuffer buffer;
  while (!is.eof() && !context.reconstructFrame) {
    pcc::PayloadBuffer* bufferPtr = &buffer;
    readTlv(is, &buffer);

    // at end of file (or other error), flush decoder
    if (!is) { bufferPtr = nullptr; }

    if (decoder.decompress(bufferPtr, this)) {
      BOOST_THROW_EXCEPTION(std::logic_error("decompress point cloud error"));
    }

    if (!bufferPtr) { break; }
  }

  assert(context.reconstructFrame);

  contextPtr_ = nullptr;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCDecoderTMC3<PointT>::convertToPCL(JPCCCoderContext<PointT>& context) {
  auto* reconstructFrame    = static_cast<pcc::PCCPointSet3*>(context.reconstructFrame.get());
  auto  reconstructPclFrame = make_shared<Frame<PointT>>();

  reconstructPclFrame->resize(reconstructFrame->getPointCount());

  for (int i = 0; i < reconstructPclFrame->size(); i++) {
    reconstructPclFrame->at(i) =
        PointT((*reconstructFrame)[i].x(), (*reconstructFrame)[i].y(), (*reconstructFrame)[i].z());
  }

  context.reconstructPclFrame         = reconstructPclFrame;
  context.reconstructPclFrame->header = context.pclFrame->header;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCDecoderTMC3<PointT>::onOutputCloud(const pcc::CloudFrame& frame) {
  if (contextPtr_) {
    auto reconstructFrame         = make_shared<pcc::PCCPointSet3>(frame.cloud);
    contextPtr_->reconstructFrame = reconstructFrame;
  }
}

}  // namespace jpcc::coder