#include <chrono>
#include <iostream>
#include <vector>

#include <jpcc/common/ParameterParser.h>
#include <jpcc/io/PlyIO.h>
#include <jpcc/io/Reader.h>
#include <jpcc/process/PreProcessor.h>
#include <jpcc/segmentation/JPCCSegmentationAdapter.h>

#include "AppParameter.h"

using namespace std;
using namespace std::chrono;
using namespace pcc;
using namespace pcc::chrono;
using namespace jpcc;
using namespace jpcc::io;
using namespace jpcc::process;
using namespace jpcc::segmentation;

using PointEncode = pcl::PointXYZI;
using PointOutput = pcl::PointXYZ;

void encode(const AppParameter& parameter) {
  DatasetReader<PointEncode>::Ptr    reader = newReader<PointEncode>(parameter.inputReader, parameter.inputDataset);
  PreProcessor<PointEncode>          preProcessor(parameter.preProcess);
  JPCCSegmentation<PointEncode>::Ptr gmmSegmentation = JPCCSegmentationAdapter::build<PointEncode>(
      parameter.jpccGmmSegmentation, (int)parameter.inputDataset.getStartFrameNumber());

  // TODO extract JPCCMetric
  Stopwatch<steady_clock> clockLoad;
  Stopwatch<steady_clock> clockPreProcess;
  Stopwatch<steady_clock> clockBuild;
  Stopwatch<steady_clock> clockEncode;
  Stopwatch<steady_clock> clockSave;
  Stopwatch<steady_clock> clockMetric;

  uint64_t rawPoints           = 0;
  uint64_t preProcessedPoints  = 0;
  uint64_t dynamicPoints       = 0;
  uint64_t staticAddedPoints   = 0;
  uint64_t staticRemovedPoints = 0;

  {  // build gaussian mixture model
    GroupOfFrame<PointEncode> frames;
    size_t                    groupOfFramesSize = parameter.groupOfFramesSize;
    size_t                    frameNumber       = parameter.inputDataset.getStartFrameNumber();
    const size_t              endFrameNumber    = parameter.inputDataset.getEndFrameNumber();
    while (!gmmSegmentation->isBuilt() && frameNumber < endFrameNumber) {
      clockLoad.start();
      reader->loadAll(frameNumber, groupOfFramesSize, frames, parameter.parallel);
      clockLoad.stop();

      clockPreProcess.start();
      preProcessor.process(frames, nullptr, parameter.parallel);
      clockPreProcess.stop();

      clockBuild.start();
      for (const auto& frame : frames) { gmmSegmentation->appendTrainSamples(frame); }
      clockBuild.stop();

      frameNumber += groupOfFramesSize;
    }
  }

#if !defined(NDEBUG)
  size_t staticPointSize = 0;
#endif
  GroupOfFrame<PointEncode> frames;
  GroupOfFrame<PointEncode> dynamicFrames;
  GroupOfFrame<PointEncode> staticAddedFrames;
  GroupOfFrame<PointEncode> staticRemovedFrames;
  size_t                    groupOfFramesSize = parameter.groupOfFramesSize;
  size_t                    frameNumber       = parameter.inputDataset.getStartFrameNumber();
  const size_t              endFrameNumber    = parameter.inputDataset.getEndFrameNumber();

  while (frameNumber < endFrameNumber) {
    clockLoad.start();
    reader->loadAll(frameNumber, groupOfFramesSize, frames, parameter.parallel);
    clockLoad.stop();

    clockMetric.start();
    for (const auto& frame : frames) { rawPoints += frame->size(); }
    clockMetric.stop();

    clockPreProcess.start();
    preProcessor.process(frames, nullptr, parameter.parallel);
    clockPreProcess.stop();

    // TODO extract JPCCEncoder
    clockEncode.start();
    dynamicFrames.clear();
    staticAddedFrames.clear();
    staticRemovedFrames.clear();
    for (const auto& frame : frames) {
      auto dynamicFrame       = jpcc::make_shared<Frame<PointEncode>>();
      auto staticAddedFrame   = jpcc::make_shared<Frame<PointEncode>>();
      auto staticRemovedFrame = jpcc::make_shared<Frame<PointEncode>>();

#if defined(NDEBUG)
      gmmSegmentation->segmentation(frame, dynamicFrame, nullptr, staticAddedFrame, staticRemovedFrame);

#else
      auto staticFrame = jpcc::make_shared<Frame<PointEncode>>();
      gmmSegmentation->segmentation(frame, dynamicFrame, staticFrame, staticAddedFrame, staticRemovedFrame);

      staticPointSize += staticAddedFrame->size();
      staticPointSize -= staticRemovedFrame->size();
      assert(staticPointSize == staticFrame->size());
#endif

      dynamicFrames.push_back(dynamicFrame);
      staticAddedFrames.push_back(staticAddedFrame);
      staticRemovedFrames.push_back(staticRemovedFrame);
    }
    clockEncode.stop();

    clockSave.start();
    // TODO extract JPCCWriter
    savePly<PointEncode, PointOutput>(dynamicFrames, parameter.outputDataset.getFilePath(0), parameter.parallel);
    savePly<PointEncode, PointOutput>(staticAddedFrames, parameter.outputDataset.getFilePath(1), parameter.parallel);
    savePly<PointEncode, PointOutput>(staticRemovedFrames, parameter.outputDataset.getFilePath(2), parameter.parallel);
    clockSave.stop();

    clockMetric.start();
    for (const auto& frame : frames) { preProcessedPoints += frame->size(); }
    for (const auto& frame : dynamicFrames) { dynamicPoints += frame->size(); }
    for (const auto& frame : staticAddedFrames) { staticAddedPoints += frame->size(); }
    for (const auto& frame : staticRemovedFrames) { staticRemovedPoints += frame->size(); }
    clockMetric.stop();

    frameNumber += groupOfFramesSize;
  }

  cout << "\n\nMetrics:" << endl;
  cout << "  Points: " << endl;
  cout << "    Raw            : " << preProcessedPoints << endl;
  cout << "    PreProcessed   : " << preProcessedPoints << endl;
  cout << "    Dynamic        : " << dynamicPoints << endl;
  cout << "    StaticAdded    : " << staticAddedPoints << endl;
  cout << "    StaticRemoved  : " << staticRemovedPoints << endl;

  cout << "\n\nProcessing time:\n";
  cout << "  Load       : " << (float)clockLoad.count().count() / 1000000000.0 << " s" << endl;
  cout << "  PreProcess : " << (float)clockPreProcess.count().count() / 1000000000.0 << " s" << endl;
  cout << "  Build      : " << (float)clockBuild.count().count() / 1000000000.0 << " s" << endl;
  cout << "  Encode     : " << (float)clockEncode.count().count() / 1000000000.0 << " s" << endl;
  cout << "  Save       : " << (float)clockSave.count().count() / 1000000000.0 << " s" << endl;
  cout << "  Metric     : " << (float)clockMetric.count().count() / 1000000000.0 << " s" << endl;
}

int main(int argc, char* argv[]) {
  cout << "JPCC App Encoder Start" << endl;

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

    clockWall.start();
    encode(parameter);
    clockWall.stop();

    auto totalWall = duration_cast<milliseconds>(clockWall.count()).count();
    cout << "Processing time (wall): " << (float)totalWall / 1000.0 << " s\n";
    cout << "Peak memory: " << getPeakMemory() << " KB\n";
  } catch (exception& e) { cerr << e.what() << endl; }

  cout << "JPCC App Encoder End" << endl;
  return 0;
}