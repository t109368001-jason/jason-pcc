namespace jpcc::coder {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCDecoderTMC3<PointT>::JPCCDecoderTMC3(const JPCCDecoderParameter& parameter) :
    JPCCDecoder<PointT>(parameter), decoder(parameter.tmc3) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCDecoderTMC3<PointT>::convertFromPCL(JPCCDecoderContext<PointT>& context) {
  auto frame = make_shared<pcc::PCCPointSet3>();

  frame->reserve(context.pclFrame->size());

  for (int i = 0; i < frame->getPointCount(); i++) {
    (*frame)[i] =
        pcc::PCCPointSet3::PointType(context.pclFrame->at(i).x, context.pclFrame->at(i).y, context.pclFrame->at(i).z);
  }

  context.frame = frame;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCDecoderTMC3<PointT>::decode(JPCCDecoderContext<PointT>& context) {
  decoder.decompress(static_cast<pcc::PayloadBuffer*>(context.encodedFrame.get()), this);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCDecoderTMC3<PointT>::onOutputCloud(const pcc::CloudFrame& frame) {
  // TODO
}

}  // namespace jpcc::coder