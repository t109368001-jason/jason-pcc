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
                      const bool            parallel) const {
  if (datasetParam_.type != "pcap") { throw runtime_error("PcapReader: only support pcap type dataset"); }
  for (string file : datasetParam_.files) {
    string pcapPath = datasetParam_.folder + file;
    if (!filesystem::exists(pcapPath)) { throw runtime_error("PcapReader: " + pcapPath + " not found"); }
  }
  std::vector<int> datasetIndices(datasetParam_.totalFiles);
  std::generate(datasetIndices.begin(), datasetIndices.end(), [n = 0]() mutable { return n++; });
  sources.resize(datasetParam_.totalFiles);
  if (parallel) {
    std::transform(std::execution::par, datasetIndices.begin(), datasetIndices.end(), sources.begin(),
                   [this, &startFrameIndex, &groupOfFramesSize](size_t datasetIndex) {
                     return this->load(datasetIndex, startFrameIndex, groupOfFramesSize);
                   });
  } else {
    std::transform(datasetIndices.begin(), datasetIndices.end(), sources.begin(),
                   [this, &startFrameIndex, &groupOfFramesSize](size_t datasetIndex) {
                     return this->load(datasetIndex, startFrameIndex, groupOfFramesSize);
                   });
  }
}

GroupOfFrame PcapReader::load(const size_t datasetIndex,
                              const size_t startFrameIndex,
                              const size_t groupOfFramesSize,
                              const bool   parallel) const {
  GroupOfFrame frames;
  VLP16Capture capture(datasetParam_.getFilePath(datasetIndex));
  int          frameIndex = 0;

  frames.resize(groupOfFramesSize);
  frames.setStartFrameIndex(startFrameIndex);
  while (capture.isRun() && frameIndex < startFrameIndex + groupOfFramesSize) {
    if (capture.getQueueSize() > 0) {
      vector<Laser> lasers;

      capture.retrieve(lasers);
      if (frameIndex >= startFrameIndex) {
        Frame& frame = frames[frameIndex - startFrameIndex];

        frame.addPointTypes(param_.pointTypes);
        frame.resize(lasers.size());
        for (Laser& laser : lasers) { frame.add(laser); }

        cout << datasetIndex << ":" << frameIndex << " " << frame << endl;
      }
      frameIndex++;
    } else {
      this_thread::sleep_for(chrono::milliseconds(100));
    }
  }
  capture.close();
  return frames;
}

}  // namespace io
}  // namespace jpcc
