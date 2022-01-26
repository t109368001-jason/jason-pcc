#include <jpcc/io/PlyIO.h>

#include <algorithm>
#include <execution>
#include <functional>

#include <pcl/io/auto_io.h>

namespace jpcc::io {

using namespace std;

void load(
    GroupOfFrame& groupOfFrame, std::string filePath, size_t startFrameNumber, size_t endFrameNumber, bool parallel) {
  const size_t frameCount = endFrameNumber - startFrameNumber;
  groupOfFrame.clear();
  groupOfFrame.getFrames().resize(frameCount);
  vector<size_t> frameNumbers;
  for (size_t i = startFrameNumber; i < endFrameNumber; i++) { frameNumbers.push_back(i); }
  auto func = [&](const size_t frameNumber) {
    char fileName[4096];
    sprintf(fileName, filePath.c_str(), frameNumber);
    auto& frame = groupOfFrame.at(frameNumber - startFrameNumber);
    frame.reset(new Frame());

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

void save(const GroupOfFrame& groupOfFrame, std::string filePath, bool parallel) {
  auto func = [&](const Frame::Ptr& frame) {
    char fileName[4096];
    sprintf(fileName, filePath.c_str(), frame->header.seq);

    int result = pcl::io::savePLYFileASCII(string(fileName), *frame);
    assert(result != -1);
  };

  if (parallel) {
    for_each(execution::par, groupOfFrame.getFrames().begin(), groupOfFrame.getFrames().end(), func);
  } else {
    for_each(groupOfFrame.getFrames().begin(), groupOfFrame.getFrames().end(), func);
  }
}

}  // namespace jpcc::io
