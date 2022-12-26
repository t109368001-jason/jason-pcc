#include <jpcc/decoder/JPCCDecoder.h>

#include <execution>

#include <boost/range/counting_range.hpp>

namespace jpcc::decoder {

//////////////////////////////////////////////////////////////////////////////////////////////
bool JPCCDecoder::isConvertFromCoderTypeThreadSafe() { return false; }

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCDecoder::convertFromCoderType(const std::vector<std::shared_ptr<void>>& coderFrames,
                                       GroupOfFrame&                             frames,
                                       const bool                                parallel) {
  if (!parallel || !isConvertFromCoderTypeThreadSafe()) {
    for (size_t i = 0; i < frames.size(); i++) { this->convertFromCoderType(coderFrames[i], frames[i]); }
  } else {
    const auto range = boost::counting_range<size_t>(0, frames.size());
    std::for_each(std::execution::par, range.begin(), range.end(),
                  [&](const size_t& i) {  //
                    this->convertFromCoderType(coderFrames[i], frames[i]);
                  });
  }
}

}  // namespace jpcc::decoder