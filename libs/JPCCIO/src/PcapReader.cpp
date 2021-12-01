#include <jpcc/io/PcapReader.h>

#include <execution>
#include <exception>
#include <filesystem>
#include <fstream>

#include <VelodyneCapture.h>

#include <jpcc/common/Transform.h>

namespace jpcc {
namespace io {

using namespace std;
using namespace velodyne;
using namespace jpcc::common;

void PcapReader::setPcapReaderParameter(const PcapReaderParameter& param) { param_ = param; }

void PcapReader::setDatasetParameter(const DatasetParameter& datasetParam) { datasetParam_ = datasetParam; }

void PcapReader::load(vector<GroupOfFrame>& sources,
                      const size_t          startFrameIndex,
                      const size_t          groupOfFramesSize,
                      const bool            parallel) {
  if (datasetParam_.type != "pcap") { throw runtime_error("PcapReader: only support pcap type dataset"); }
  std::vector<int> datasetIndices(datasetParam_.totalFiles);
  std::generate(datasetIndices.begin(), datasetIndices.end(), [n = 0]() mutable { return n++; });
  for (size_t datasetIndex : datasetIndices) {
    string pcapPath = datasetParam_.getFilePath(datasetIndex);
    if (!filesystem::exists(pcapPath)) { throw runtime_error("PcapReader: " + pcapPath + " not found"); }
  }
  open(startFrameIndex);

  sources.resize(datasetParam_.totalFiles);
  if (parallel) {
    std::transform(std::execution::par, datasetIndices.begin(), datasetIndices.end(), sources.begin(),
                   [this, &startFrameIndex, &groupOfFramesSize, &parallel](size_t datasetIndex) {
                     return this->load(this->captures[datasetIndex], startFrameIndex, groupOfFramesSize, parallel);
                   });
  } else {
    std::transform(datasetIndices.begin(), datasetIndices.end(), sources.begin(),
                   [this, &startFrameIndex, &groupOfFramesSize, &parallel](size_t datasetIndex) {
                     return this->load(this->captures[datasetIndex], startFrameIndex, groupOfFramesSize, parallel);
                   });
  }
}

void PcapReader::open(const size_t startFrameIndex) {
  if (captures.size() == datasetParam_.totalFiles) {
    bool needReset = false;
    for (size_t i = 0; i < captures.size(); i++) {
      string pcapPath = datasetParam_.getFilePath(i);
      if (!captures[i]->isRun()) {
        needReset = true;
        break;
      }
      if (captures[i]->getFilename() != pcapPath) {
        needReset = true;
        break;
      }
      if (captures[i]->getCurrentFrameIndex() > startFrameIndex) {
        needReset = true;
        break;
      }
    }
    if (!needReset) { return; }
  }

  for (size_t i = 0; i < captures.size(); i++) { captures[i]->close(); }
  captures.clear();
  for (size_t i = 0; i < datasetParam_.totalFiles; i++) {
    shared_ptr<VLP16Capture> capture(new VLP16Capture(datasetParam_.getFilePath(i), param_.bufferSize));
    captures.push_back(capture);
  }
}

GroupOfFrame PcapReader::load(const shared_ptr<VLP16Capture> capture,
                              const size_t                   startFrameIndex,
                              const size_t                   groupOfFramesSize,
                              const bool                     parallel) {
  assert(capture->isRun());
  assert(groupOfFramesSize > 0);
  GroupOfFrame frames;
  int          currentFrameIndex = capture->getCurrentFrameIndex();
  assert(currentFrameIndex <= startFrameIndex);
  frames.resize(groupOfFramesSize);
  frames.setStartFrameIndex(startFrameIndex);
  while (capture->isRun() && currentFrameIndex < startFrameIndex + groupOfFramesSize) {
    if (capture->getQueueSize() > 0) {
      vector<Laser> lasers;

      capture->retrieve(lasers);
      if (currentFrameIndex >= startFrameIndex) {
        Frame& frame = frames[currentFrameIndex - startFrameIndex];

        frame.addPointTypes(param_.pointTypes);
        frame.resize(lasers.size());
        for (Laser& laser : lasers) { frame.add(laser); }

        cout << capture->getFilename() << ":" << currentFrameIndex << " " << frame << endl;
      }
      currentFrameIndex = capture->getCurrentFrameIndex();
    } else {
      // this_thread::sleep_for(chrono::milliseconds(100));
    }
  }
  assert(currentFrameIndex >= startFrameIndex);
  frames.resize(currentFrameIndex - startFrameIndex);
  return frames;
}

}  // namespace io
}  // namespace jpcc
