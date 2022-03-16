#pragma once

#include <algorithm>
#include <execution>
#include <functional>

#include <pcl/io/auto_io.h>

namespace jpcc::io {

using namespace std;

template <typename PointT>
void loadPly(
    GroupOfFrame<PointT>& frames, std::string filePath, size_t startFrameNumber, size_t endFrameNumber, bool parallel) {
  const size_t frameCount = endFrameNumber - startFrameNumber;
  frames.clear();
  frames.resize(frameCount);
  vector<size_t> frameNumbers;
  for (size_t i = startFrameNumber; i < endFrameNumber; i++) { frameNumbers.push_back(i); }
  auto func = [&](const size_t frameNumber) {
    char fileName[4096];
    sprintf(fileName, filePath.c_str(), frameNumber);
    auto& frame = frames.at(frameNumber - startFrameNumber);
    frame.reset(new Frame<PointT>());

    int result = pcl::io::load(string(fileName), *frame);
    assert(result != -1);
    if (frame->header.seq == 0) { frame->header.seq = frameNumber; }
  };

  if (parallel) {
    for_each(execution::par, frameNumbers.begin(), frameNumbers.end(), func);
  } else {
    for_each(frameNumbers.begin(), frameNumbers.end(), func);
  }
}

template <typename PointT>
void savePly(const GroupOfFrame<PointT>& frames, std::string filePath, bool parallel) {
  auto func = [&](const FramePtr<PointT>& frame) {
    char fileName[4096];
    sprintf(fileName, filePath.c_str(), frame->header.seq);

    int result = pcl::io::savePLYFileASCII(string(fileName), *frame);
    assert(result != -1);
  };

  if (parallel) {
    for_each(execution::par, frames.begin(), frames.end(), func);
  } else {
    for_each(frames.begin(), frames.end(), func);
  }
}

}  // namespace jpcc::io
