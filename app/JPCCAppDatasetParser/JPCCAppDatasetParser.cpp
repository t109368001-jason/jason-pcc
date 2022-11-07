#include <chrono>
#include <iostream>
#include <vector>

#include <jpcc/common/ParameterParser.h>
#include <jpcc/io/Reader.h>
#include <jpcc/io/PlyIO.h>
#include <jpcc/process/PreProcessor.h>
#include <jpcc/process/JPCCNormalEstimation.h>

#include "AppParameter.h"

using namespace std;
using namespace std::chrono;
using namespace pcc;
using namespace jpcc;
using namespace jpcc::io;
using namespace jpcc::process;

template <typename PointT>
void parse(const AppParameter& parameter, Stopwatch& clock) {
  const typename DatasetReader::Ptr reader = newReader(parameter.inputReader, parameter.inputDataset);
  PreProcessor                      preProcessor(parameter.preProcess);

  GroupOfFrame frames;
  size_t       frameNumber    = parameter.inputDataset.getStartFrameNumber();
  const size_t endFrameNumber = parameter.inputDataset.getEndFrameNumber();
  while (frameNumber < endFrameNumber) {
    size_t groupOfFramesSize = std::min(parameter.groupOfFramesSize, endFrameNumber - frameNumber);
    clock.start();
    reader->loadAll(frameNumber, groupOfFramesSize, frames, parameter.parallel);
    if constexpr (!pcl::traits::has_intensity_v<PointT>) {
      for (const auto& frame : frames) { frame->removeReflectances(); }
    }
    preProcessor.process(frames, nullptr, parameter.parallel);
    if constexpr (pcl::traits::has_normal_v<PointT>) {
      auto normalEstimation = jpcc::make_shared<JPCCNormalEstimation>(parameter.jpccNormalEstimation);
      normalEstimation->computeInPlaceAll(frames, parameter.parallel);
    } else {
      for (const auto& frame : frames) { frame->removeNormal(); }
    }
    clock.stop();

    savePly(frames, parameter.outputDataset.getFilePath(), 0);

    frameNumber += groupOfFramesSize;
  }
}

void parse(const AppParameter& parameter, Stopwatch& clock) {
  if (parameter.outputPointType == "PointXYZ") {
    parse<pcl::PointXYZ>(parameter, clock);
  } else if (parameter.outputPointType == "PointNormal") {
    parse<pcl::PointNormal>(parameter, clock);
  } else if (parameter.outputPointType == "PointXYZI") {
    parse<pcl::PointXYZI>(parameter, clock);
  } else if (parameter.outputPointType == "PointXYZINormal") {
    parse<pcl::PointXYZINormal>(parameter, clock);
  } else {
    BOOST_THROW_EXCEPTION(std::logic_error("invalid point type"));
  }
}

int main(int argc, char* argv[]) {
  cout << "JPCC App Dataset Parser Start" << endl;

  AppParameter parameter;
  try {
    ParameterParser pp;
    pp.add(parameter);
    if (!pp.parse(argc, argv)) { return 1; }
    cout << parameter << endl;
  } catch (exception& e) {
    cerr << e.what() << endl;
    return 1;
  }

  try {
    // Timers to count elapsed wall/user time
    Stopwatch clockWall;
    Stopwatch clockUser;

    clockWall.start();
    parse(parameter, clockUser);
    clockWall.stop();

    auto totalWall = duration_cast<milliseconds>(clockWall.count()).count();
    auto totalUser = duration_cast<milliseconds>(clockUser.count()).count();
    cout << "Processing time (wall): " << (float)totalWall / 1000.0 << " s\n";
    cout << "Processing time (user): " << (float)totalUser / 1000.0 << " s\n";
    cout << "Peak memory: " << getPeakMemory() << " KB\n";
  } catch (exception& e) { cerr << e.what() << endl; }

  cout << "JPCC App Dataset Parser End" << endl;
  return 0;
}