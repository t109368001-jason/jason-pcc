#include <jpcc/io/PlyIO.h>

#include <algorithm>
#include <execution>
#include <filesystem>

#include <boost/range/counting_range.hpp>

using namespace std::filesystem;

namespace jpcc::io {

//////////////////////////////////////////////////////////////////////////////////////////////
void loadPly(GroupOfFrame&      frames,
             const std::string& filePath,
             const size_t       startFrameNumber,
             const size_t       endFrameNumber,
             const bool         parallel) {
  const size_t frameCount = endFrameNumber - startFrameNumber;
  frames.clear();
  frames.resize(frameCount);
  std::vector<size_t> frameNumbers;
  for (size_t i = startFrameNumber; i < endFrameNumber; i++) {
    frameNumbers.push_back(i);
  }
  auto func = [&](const size_t frameNumber) {
    char fileName[4096];
    sprintf(fileName, filePath.c_str(), frameNumber);
    if (exists(fileName)) {
      auto& frame = frames[frameNumber - startFrameNumber];
      frame       = jpcc::make_shared<Frame>();
      frame->setFrameNumber((Index)frameNumber);
      const bool result = frame->read(std::string(fileName), true);
      THROW_IF_NOT(result);
    }
  };

  if (!parallel) {
    std::for_each(frameNumbers.begin(), frameNumbers.end(), func);
  } else {
    std::for_each(std::execution::par, frameNumbers.begin(), frameNumbers.end(), func);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void savePly(const GroupOfFrame& frames, const std::string& filePath, const bool parallel) {
  auto func = [&](const FramePtr& frame) {
    if (!frame || frame->getPointCount() == 0) {
      return;
    }
    char fileName[4096];
    sprintf(fileName, filePath.c_str(), frame->getFrameNumber());
    const bool result = frame->write(std::string(fileName), true);
    THROW_IF_NOT(result);
  };

  if (!parallel) {
    for (const auto& frame : frames) {
      func(frame);
    }
  } else {
    const auto range = boost::counting_range<size_t>(0, frames.size());
    std::for_each(std::execution::par, range.begin(), range.end(),
                  [&](const size_t& i) {  //
                    func(frames[i]);
                  });
  }
}

}  // namespace jpcc::io
