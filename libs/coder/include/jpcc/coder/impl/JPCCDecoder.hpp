namespace jpcc::coder {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCDecoder<PointT>::JPCCDecoder(const JPCCDecoderParameter& parameter) : parameter_(parameter) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCDecoder<PointT>::convertFromPCL(JPCCDecoderContext<PointT>& context) {
  context.frame = context.pclFrame;
}

}  // namespace jpcc::coder