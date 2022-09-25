namespace jpcc::coder {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCDecoder<PointT>::JPCCDecoder(JPCCDecoderParameter parameter) : parameter_(parameter) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool JPCCDecoder<PointT>::isThreadSafe() {
  return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCDecoder<PointT>::decode(std::istream& is, std::vector<shared_ptr<void>>& reconstructFrames, bool parallel) {
  for (auto& reconstructFrame : reconstructFrames) { this->decode(is, reconstructFrame); }
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