#include <sstream>

#include <io_tlv.h>

namespace jpcc::decoder {

//////////////////////////////////////////////////////////////////////////////////////////////
PCCTMC3Decoder3LambdaCallbacks::PCCTMC3Decoder3LambdaCallbacks(
    const std::function<void(const pcc::CloudFrame& frame)>& onOutputCloud) :
    onOutputCloud_(onOutputCloud) {}

//////////////////////////////////////////////////////////////////////////////////////////////
void PCCTMC3Decoder3LambdaCallbacks::onOutputCloud(const pcc::CloudFrame& frame) { onOutputCloud_(frame); }

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool JPCCDecoderTMC3<PointT>::isConvertToPCLThreadSafe() {
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCDecoderTMC3<PointT>::decode(std::istream& is, shared_ptr<void>& reconstructFrame) {
  std::function<void(const pcc::CloudFrame& frame)> onOutputCloud = [&](const pcc::CloudFrame& frame) {
    reconstructFrame       = make_shared<pcc::PCCPointSet3>(frame.cloud);
    auto reconstructFrame_ = std::static_pointer_cast<pcc::PCCPointSet3>(reconstructFrame);
    for (size_t i = 0; i < frame.cloud.getPointCount(); i++) {  //
      (*reconstructFrame_)[i] += frame.outputOrigin;
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
      reconstructFrame = make_shared<pcc::PCCPointSet3>();
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

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCDecoderTMC3<PointT>::convertToPCL(shared_ptr<void>& reconstructFrame, FramePtr<PointT>& reconstructPclFrame) {
  auto reconstructFrame_ = std::static_pointer_cast<pcc::PCCPointSet3>(reconstructFrame);
  if (!reconstructFrame_) { return; }

  reconstructPclFrame = jpcc::make_shared<Frame<PointT>>();
  reconstructPclFrame->resize(reconstructFrame_->getPointCount());

  for (int i = 0; i < reconstructPclFrame->size(); i++) {
    (*reconstructPclFrame)[i] =
        PointT((*reconstructFrame_)[i].x(), (*reconstructFrame_)[i].y(), (*reconstructFrame_)[i].z());
  }
  assert(reconstructFrame_->getPointCount() == reconstructPclFrame->size());
  std::cout << __FUNCTION__ << "() "
            << "tmc3Points=" << reconstructFrame_->getPointCount() << ", "
            << "pclPoints=" << reconstructPclFrame->size() << std::endl;
}

}  // namespace jpcc::decoder