namespace jpcc::coder {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCEncoder<PointT>::JPCCEncoder(const JPCCEncoderParameter& parameter) : parameter_(parameter) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCEncoder<PointT>::convertFromPCL(JPCCEncoderContext<PointT>& context) {
  context.frame = context.pclFrame;
}

}  // namespace jpcc::coder