namespace jpcc::coder {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCEncoderTMC3<PointT>::JPCCEncoderTMC3(const JPCCEncoderParameter& parameter) :
    JPCCEncoder<PointT>(parameter), encoder() {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCEncoderTMC3<PointT>::convertFromPCL(JPCCEncoderContext<PointT>& context) {
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
void JPCCEncoderTMC3<PointT>::encode(JPCCEncoderContext<PointT>& context) {
  encoder.compress(*static_cast<pcc::PCCPointSet3*>(context.frame.get()),
                   static_cast<pcc::EncoderParams*>(&this->parameter_.tmc3), this, nullptr);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCEncoderTMC3<PointT>::onOutputBuffer(const pcc::PayloadBuffer& buffer) {
  // TODO
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCEncoderTMC3<PointT>::onPostRecolour(const pcc::PCCPointSet3& set3) {
  // TODO
}

}  // namespace jpcc::coder