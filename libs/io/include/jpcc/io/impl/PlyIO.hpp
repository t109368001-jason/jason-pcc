#include <algorithm>
#include <execution>
#include <filesystem>

#include <pcl/io/auto_io.h>

using namespace std::filesystem;

namespace jpcc::io {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void loadPly(GroupOfFrame<PointT>& frames,
             const std::string&    filePath,
             const size_t          startFrameNumber,
             const size_t          endFrameNumber,
             const bool            parallel) {
  const size_t frameCount = endFrameNumber - startFrameNumber;
  frames.clear();
  frames.resize(frameCount);
  std::vector<size_t> frameNumbers;
  for (size_t i = startFrameNumber; i < endFrameNumber; i++) { frameNumbers.push_back(i); }
  auto func = [&](const size_t frameNumber) {
    char fileName[4096];
    sprintf(fileName, filePath.c_str(), frameNumber);
    if (exists(fileName)) {
      auto& frame      = frames.at(frameNumber - startFrameNumber);
      frame            = jpcc::make_shared<Frame<PointT>>();
      const int result = pcl::io::load(std::string(fileName), *frame);
      assert(result != -1);
      if (frame->header.seq == 0) { frame->header.seq = frameNumber; }
    }
  };

  if (parallel) {
    std::for_each(std::execution::par, frameNumbers.begin(), frameNumbers.end(), func);
  } else {
    std::for_each(frameNumbers.begin(), frameNumbers.end(), func);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void savePly(const GroupOfFrame<PointT>& frames, const std::string& filePath, const bool parallel) {
  auto func = [&](const FramePtr<PointT>& frame) {
    if (!frame || frame->empty()) { return; }
    char fileName[4096];
    sprintf(fileName, filePath.c_str(), frame->header.seq);

    const int result = pcl::io::savePLYFileASCII(std::string(fileName), *frame);
    assert(result != -1);
  };

  if (parallel) {
    std::for_each(std::execution::par, frames.begin(), frames.end(), func);
  } else {
    std::for_each(frames.begin(), frames.end(), func);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointIn, typename PointOut>
void savePly(const GroupOfFrame<PointIn>& frames, const std::string& filePath, const bool parallel) {
  if constexpr (std::is_same_v<PointIn, PointOut>) {
    savePly<PointIn>(frames, filePath, parallel);
  } else {
    auto func = [&](const FramePtr<PointIn>& frame) {
      if (!frame || frame->empty()) { return; }
      auto outputFrame = jpcc::make_shared<Frame<PointOut>>();
      pcl::copyPointCloud(*frame, *outputFrame);
      char fileName[4096];
      sprintf(fileName, filePath.c_str(), frame->header.seq);

      const int result = pcl::io::savePLYFileASCII(std::string(fileName), *outputFrame);
      assert(result != -1);
    };

    if (parallel) {
      std::for_each(std::execution::par, frames.begin(), frames.end(), func);
    } else {
      std::for_each(frames.begin(), frames.end(), func);
    }
  }
}

}  // namespace jpcc::io
