#include <execution>

#include <boost/range/counting_range.hpp>

namespace jpcc::coder {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCEncoder<PointT>::JPCCEncoder(JPCCEncoderParameter parameter) : parameter_(parameter) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool JPCCEncoder<PointT>::isConvertFromPCLThreadSafe() {
  return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool JPCCEncoder<PointT>::isEncodeThreadSafe() {
  return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCEncoder<PointT>::convertFromPCL(const GroupOfFrame<PointT>&    pclFrames,
                                         std::vector<shared_ptr<void>>& frames,
                                         const bool                     parallel) {
  frames.clear();
  frames.resize(pclFrames.size());
  if (!parallel || !isConvertFromPCLThreadSafe()) {
    for (size_t i = 0; i < pclFrames.size(); i++) { this->convertFromPCL(pclFrames[i], frames[i]); }
  } else {
    const auto range = boost::counting_range<size_t>(0, pclFrames.size());
    std::for_each(std::execution::par, range.begin(), range.end(),
                  [&](const size_t& i) {  //
                    this->convertFromPCL(pclFrames[i], frames[i]);
                  });
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCEncoder<PointT>::encode(const std::vector<shared_ptr<void>>& frames,
                                 std::vector<char>&                   encodedBytes,
                                 const bool                           parallel) {
  if (!parallel || !isEncodeThreadSafe()) {
    for (size_t i = 0; i < frames.size(); i++) { this->encode(frames[i], encodedBytes); }
  } else {
    std::vector<std::vector<char>> encodedFramesBytes(frames.size());
    const auto                     range = boost::counting_range<size_t>(0, frames.size());
    std::for_each(std::execution::par, range.begin(), range.end(),
                  [&](const size_t& i) {  //
                    this->encode(frames[i], encodedFramesBytes[i]);
                  });

    for (auto& _encodedBytes : encodedFramesBytes) {
      std::copy(std::move_iterator(_encodedBytes.begin()), std::move_iterator(_encodedBytes.end()),
                std::back_insert_iterator(encodedBytes));
    }
  }
}

}  // namespace jpcc::coder