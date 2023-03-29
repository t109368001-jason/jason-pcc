#include <jpcc/encoder/JPCCEncoder.h>

#include <execution>

#include <boost/range/counting_range.hpp>
#include <utility>

namespace jpcc::encoder {

//////////////////////////////////////////////////////////////////////////////////////////////
bool JPCCEncoder::isConvertToCoderTypeThreadSafe() {
  return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool JPCCEncoder::isEncodeThreadSafe() {
  return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoder::convertToCoderType(const GroupOfFrame& frames, CoderGroupOfFrame& coderFrames, const bool parallel) {
  if (!parallel || !isConvertToCoderTypeThreadSafe()) {
    for (size_t i = 0; i < frames.size(); i++) {
      this->convertToCoderType(frames[i], coderFrames[i]);
    }
  } else {
    const auto range = boost::counting_range<size_t>(0, frames.size());
    std::for_each(std::execution::par, range.begin(), range.end(),
                  [&](const size_t& i) {  //
                    this->convertToCoderType(frames[i], coderFrames[i]);
                  });
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoder::encode(const CoderGroupOfFrame& coderFrames, std::ostream& os, const bool parallel) {
  if (!parallel || !isEncodeThreadSafe()) {
    for (const auto& coderFrame : coderFrames) {
      this->encode(coderFrame, os);
    }
  } else {
    std::vector<std::stringstream> encodedBytesVector(coderFrames.size());
    const auto                     range = boost::counting_range<size_t>(0, coderFrames.size());
    std::for_each(std::execution::par, range.begin(), range.end(),
                  [&](const size_t& i) {  //
                    this->encode(coderFrames[i], encodedBytesVector[i]);
                  });
    for (size_t i = 0; i < encodedBytesVector.size(); i++) {
      os << encodedBytesVector[i].str();
    }
  }
}

}  // namespace jpcc::encoder