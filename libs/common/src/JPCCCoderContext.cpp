#include <jpcc/common/JPCCCoderContext.h>

namespace jpcc {

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCCoderContext::JPCCCoderContext(const std::string& compressedStreamPath) :
    compressedStreamPath_(compressedStreamPath), header_({0}), startFrameNumber_(0), frames_(), coderFrames_(), fs_() {
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCCoderContext::writeHeader(double resolution, CoderBackendType backendType) {
  header_.resolution  = resolution;
  header_.backendType = backendType;
  fs_.open(compressedStreamPath_, std::ios::binary | std::ios::out);
  header_.write(fs_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCCoderContext::readHeader() {
  fs_.open(compressedStreamPath_, std::ios::binary | std::ios::in);
  header_.read(fs_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCCoderContext::clear() {
  frames_.clear();
  coderFrames_.clear();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCCoderContext::flush() {
  fs_.flush();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCCoderContext::ifsSeekgEnd() {
  fs_.close();
  fs_.open(compressedStreamPath_, std::ios::binary | std::ios::in);
  fs_.seekg(0, std::ios::end);
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool JPCCCoderContext::eof() const {
  return fs_.eof();
}

}  // namespace jpcc