#include <chrono>
#include <iostream>
#include <vector>

#include <jpcc/common/ParameterParser.h>
#include <jpcc/io/PlyIO.h>
#include <jpcc/io/Reader.h>
#include <jpcc/process/PreProcessor.h>
#include <jpcc/process/JPCCNormalEstimation.h>
#include <jpcc/segmentation/JPCCGMMSegmentation.h>
#include <jpcc/visualization/JPCCVisualizer.h>

#include "AppParameter.h"

using namespace std;
using namespace std::chrono;
using namespace pcc;
using namespace pcc::chrono;
using namespace jpcc;
using namespace jpcc::io;
using namespace jpcc::process;
using namespace jpcc::segmentation;
using namespace jpcc::visualization;

void parse(const AppParameter& parameter, StopwatchUserTime& clock) {
  const typename DatasetReader::Ptr reader = newReader(parameter.inputReader, parameter.inputDataset);
  PreProcessor                      preProcessor(parameter.preProcess);
  auto                normalEstimation = jpcc::make_shared<JPCCNormalEstimation>(parameter.jpccNormalEstimation);
  auto                gmmSegmentation  = jpcc::make_shared<JPCCGMMSegmentation>(parameter.jpccGmmSegmentation);
  JPCCVisualizer::Ptr viewer;

  const string dynamicId = "dynamic";
  const string staticId  = "static";

  if (!parameter.headless) {
    viewer = jpcc::make_shared<JPCCVisualizer>(parameter.visualizerParameter);
    viewer->addParameter(parameter);
    viewer->setColor(dynamicId, 1.0, 1.0, 1.0);
    viewer->setColor(staticId, "z");
  }

  atomic_bool              run(true);
  jpcc::shared_ptr<thread> viewerThread;
  if (viewer) {
    viewerThread = jpcc::make_shared<thread>([&]() {
      while (!viewer->wasStopped() && run) { viewer->spinOnce(1000); }
    });
  }

  {
    GroupOfFrame frames;
    size_t       groupOfFramesSize = 32;
    size_t       frameNumber       = parameter.inputDataset.getStartFrameNumber();
    const size_t endFrameNumber    = frameNumber + gmmSegmentation->getNTrain();
    while (frameNumber < endFrameNumber) {
      size_t groupOfFramesSize_ = endFrameNumber - frameNumber;
      if (groupOfFramesSize_ < groupOfFramesSize) { groupOfFramesSize = groupOfFramesSize_; }

      clock.start();
      reader->loadAll(frameNumber, groupOfFramesSize, frames, parameter.parallel);
      preProcessor.process(frames, nullptr, parameter.parallel);
      // normalEstimation->computeInPlaceAll(frames, parameter.parallel);
      clock.stop();

      gmmSegmentation->appendTrainSamples(frames);

      frameNumber += groupOfFramesSize;
    }
  }
  gmmSegmentation->build();

  {
    GroupOfFrame frames;
    GroupOfFrame dynamicFrames;
    GroupOfFrame staticFrames;
    size_t       groupOfFramesSize = 32;
    size_t       frameNumber       = parameter.inputDataset.getStartFrameNumber();
    const size_t endFrameNumber    = frameNumber + gmmSegmentation->getNTrain();
    while (frameNumber < endFrameNumber) {
      size_t groupOfFramesSize_ = endFrameNumber - frameNumber;
      if (groupOfFramesSize_ < groupOfFramesSize) { groupOfFramesSize = groupOfFramesSize_; }

      clock.start();
      reader->loadAll(frameNumber, groupOfFramesSize, frames, parameter.parallel);
      preProcessor.process(frames, nullptr, parameter.parallel);
      // normalEstimation->computeInPlaceAll(frames, parameter.parallel);
      clock.stop();

      dynamicFrames.clear();
      staticFrames.clear();
      for (const auto& frame : frames) {
        auto dynamicFrame = jpcc::make_shared<Frame>();
        auto staticFrame  = jpcc::make_shared<Frame>();
        gmmSegmentation->segmentation(frame, dynamicFrame, staticFrame);
        dynamicFrames.push_back(dynamicFrame);
        staticFrames.push_back(staticFrame);
      }

      if (viewer) { viewer->enqueue(GroupOfFrameMap{{dynamicId, dynamicFrames}, {staticId, staticFrames}}); }

      savePly(dynamicFrames, parameter.outputDataset.getFilePath(0), parameter.parallel);
      savePly(staticFrames, parameter.outputDataset.getFilePath(1), parameter.parallel);

      frameNumber += groupOfFramesSize;
    }
  }
  run = false;
  if (viewerThread && viewerThread->joinable()) { viewerThread->join(); }
}

int main(int argc, char* argv[]) {
  cout << "JPCC App Segmentation Start" << endl;

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
    Stopwatch<steady_clock> clockWall;
    StopwatchUserTime       clockUser;

    clockWall.start();
    parse(parameter, clockUser);
    clockWall.stop();

    auto totalWall      = duration_cast<milliseconds>(clockWall.count()).count();
    auto totalUserSelf  = duration_cast<milliseconds>(clockUser.self.count()).count();
    auto totalUserChild = duration_cast<milliseconds>(clockUser.children.count()).count();
    cout << "Processing time (wall): " << (float)totalWall / 1000.0 << " s\n";
    cout << "Processing time (user.self): " << (float)totalUserSelf / 1000.0 << " s\n";
    cout << "Processing time (user.children): " << (float)totalUserChild / 1000.0 << " s\n";
    cout << "Peak memory: " << getPeakMemory() << " KB\n";
  } catch (exception& e) { cerr << e.what() << endl; }

  cout << "JPCC App Segmentation End" << endl;
  return 0;
}