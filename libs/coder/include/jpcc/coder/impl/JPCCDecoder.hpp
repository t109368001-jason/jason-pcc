namespace jpcc::coder {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCDecoder<PointT>::JPCCDecoder(const JPCCDecoderParameter& parameter) : parameter_(parameter) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool JPCCDecoder<PointT>::isThreadSafe() {
  return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCDecoder<PointT>::decode(const std::vector<std::vector<char>>& encodedFramesBytes,
                                 std::vector<shared_ptr<void>>&        reconstructFrames,
                                 bool                                  parallel) {
  if (!parallel || !isThreadSafe()) {
    for (size_t i = 0; i < encodedFramesBytes.size(); i++) {
      this->decode(encodedFramesBytes.at(i), reconstructFrames.at(i));
    }
  } else {
    const auto range = boost::counting_range<size_t>(0, encodedFramesBytes.size());
    std::for_each(std::execution::par, range.begin(), range.end(),
                  [&](const size_t& i) {  //
                    this->decode(encodedFramesBytes.at(i), reconstructFrames.at(i));
                  });
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCDecoder<PointT>::convertToPCL(std::vector<shared_ptr<void>>& reconstructFrames,
                                       GroupOfFrame<PointT>&          reconstructPclFrames,
                                       bool                           parallel) {
  if (!parallel || !isThreadSafe()) {
    for (size_t i = 0; i < reconstructFrames.size(); i++) {
      this->convertToPCL(reconstructFrames.at(i), reconstructPclFrames.at(i));
    }
  } else {
    const auto range = boost::counting_range<size_t>(0, reconstructFrames.size());
    std::for_each(std::execution::par, range.begin(), range.end(),
                  [&](const size_t& i) {  //
                    this->convertToPCL(reconstructFrames.at(i), reconstructPclFrames.at(i));
                  });
  }
}

}  // namespace jpcc::coder