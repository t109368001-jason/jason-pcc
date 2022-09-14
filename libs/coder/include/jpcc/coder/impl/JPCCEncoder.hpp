#include <execution>

#include <boost/range/counting_range.hpp>

namespace jpcc::coder {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCEncoder<PointT>::JPCCEncoder(const JPCCEncoderParameter& parameter) : parameter_(parameter) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool JPCCEncoder<PointT>::isThreadSafe() {
  return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCEncoder<PointT>::convertFromPCL(const GroupOfFrame<PointT>&    pclFrames,
                                         std::vector<shared_ptr<void>>& frames,
                                         const bool                     parallel) {
  if (!parallel || !isThreadSafe()) {
    for (size_t i = 0; i < pclFrames.size(); i++) { this->convertFromPCL(pclFrames.at(i), frames.at(i)); }
  } else {
    const auto range = boost::counting_range<size_t>(0, pclFrames.size());
    std::for_each(std::execution::par, range.begin(), range.end(),
                  [&](const size_t& i) {  //
                    this->convertFromPCL(pclFrames.at(i), frames.at(i));
                  });
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCEncoder<PointT>::encode(const std::vector<shared_ptr<void>>& frames,
                                 std::vector<std::vector<char>>&      encodedFramesBytes,
                                 const bool                           parallel) {
  if (!parallel || !isThreadSafe()) {
    for (size_t i = 0; i < frames.size(); i++) { this->encode(frames.at(i), encodedFramesBytes.at(i)); }
  } else {
    const auto range = boost::counting_range<size_t>(0, frames.size());
    std::for_each(std::execution::par, range.begin(), range.end(),
                  [&](const size_t& i) {  //
                    this->encode(frames.at(i), encodedFramesBytes.at(i));
                  });
  }
}

}  // namespace jpcc::coder