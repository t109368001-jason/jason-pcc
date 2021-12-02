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

PcapReader::PcapReader(const DatasetParameter& datasetParam, const PcapReaderParameter& param) :
    DatasetReader(datasetParam), param_(param), captures(datasetParam_.totalFiles) {
  assert(datasetParam_.type == "pcap");
}

void PcapReader::load(const size_t  datasetIndex,
                      GroupOfFrame& frames,
                      const size_t  startFrameIndex,
                      const size_t  groupOfFramesSize,
                      const bool    parallel) {
  assert(groupOfFramesSize > 0);
  open(datasetIndex, startFrameIndex);
  const shared_ptr<VLP16Capture>& capture = captures[datasetIndex];
  assert(capture->getCurrentFrameIndex() <= startFrameIndex);

  int currentFrameIndex = capture->getCurrentFrameIndex();
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
}

const PcapReaderParameter& PcapReader::getPcapReaderParameter() { return param_; }

void PcapReader::open(const size_t datasetIndex, const size_t startFrameIndex) {
  shared_ptr<VLP16Capture>& capture  = captures[datasetIndex];
  string                    pcapPath = datasetParam_.getFilePath(datasetIndex);

  if (!capture) {
    reset(datasetIndex);
  } else if (!capture->isRun()) {
    reset(datasetIndex);
  } else if (capture->getFilename() != pcapPath) {
    reset(datasetIndex);
  } else if (capture->getCurrentFrameIndex() > startFrameIndex) {
    reset(datasetIndex);
  }
}

void PcapReader::reset(const size_t datasetIndex) {
  shared_ptr<VLP16Capture>& capture  = captures[datasetIndex];
  string                    pcapPath = datasetParam_.getFilePath(datasetIndex);

  if (capture) { capture->close(); }
  capture.reset(new VLP16Capture(pcapPath, param_.bufferSize));
}

}  // namespace io
}  // namespace jpcc
