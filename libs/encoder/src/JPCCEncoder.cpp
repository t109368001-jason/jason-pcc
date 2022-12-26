#include <jpcc/encoder/JPCCEncoder.h>

#include <execution>

#include <boost/range/counting_range.hpp>
#include <utility>

namespace jpcc::encoder {

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCEncoder::JPCCEncoder(JPCCEncoderParameter parameter) : parameter_(std::move(parameter)) {}

//////////////////////////////////////////////////////////////////////////////////////////////
bool JPCCEncoder::isConvertToCoderTypeThreadSafe() { return false; }

//////////////////////////////////////////////////////////////////////////////////////////////
bool JPCCEncoder::isEncodeThreadSafe() { return false; }

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoder::convertToCoderType(const GroupOfFrame& frames, CoderGroupOfFrame& coderFrames, const bool parallel) {
  if (!parallel || !isConvertToCoderTypeThreadSafe()) {
    for (size_t i = 0; i < frames.size(); i++) { this->convertToCoderType(frames[i], coderFrames[i]); }
  } else {
    const auto range = boost::counting_range<size_t>(0, frames.size());
    std::for_each(std::execution::par, range.begin(), range.end(),
                  [&](const size_t& i) {  //
                    this->convertToCoderType(frames[i], coderFrames[i]);
                  });
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoder::encode(const CoderGroupOfFrame&        coderFrames,
                         std::vector<std::vector<char>>& encodedBytesVector,
                         const bool                      parallel) {
  if (!parallel || !isEncodeThreadSafe()) {
    for (size_t i = 0; i < coderFrames.size(); i++) { this->encode(coderFrames[i], encodedBytesVector[i]); }
  } else {
    const auto range = boost::counting_range<size_t>(0, coderFrames.size());
    std::for_each(std::execution::par, range.begin(), range.end(),
                  [&](const size_t& i) {  //
                    this->encode(coderFrames[i], encodedBytesVector[i]);
                  });
  }
}

}  // namespace jpcc::encoder