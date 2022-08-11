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

void parse(const AppParameter& parameter, StopwatchUserTime& clock) {
  DatasetReader<PointEncode>::Ptr    reader = newReader<PointEncode>(parameter.inputReader, parameter.inputDataset);
  PreProcessor<PointEncode>          preProcessor(parameter.preProcess);
  JPCCSegmentation<PointEncode>::Ptr gmmSegmentation = JPCCSegmentationAdapter::build<PointEncode>(
      parameter.jpccGmmSegmentation, parameter.inputDataset.getStartFrameNumber());

  {
    GroupOfFrame<PointEncode> frames;
    size_t                    groupOfFramesSize = parameter.groupOfFramesSize;
    size_t                    frameNumber       = parameter.inputDataset.getStartFrameNumber();
    const size_t              endFrameNumber    = parameter.inputDataset.getEndFrameNumber();
    while (!gmmSegmentation->isBuilt() && frameNumber < endFrameNumber) {
      clock.start();
      reader->loadAll(frameNumber, groupOfFramesSize, frames, parameter.parallel);
      preProcessor.process(frames, nullptr, parameter.parallel);

      for (const auto& frame : frames) { gmmSegmentation->appendTrainSamples(frame); }

      clock.stop();

      frameNumber += groupOfFramesSize;
    }
  }

  {
#if !defined(NDEBUG)
    size_t staticPointSize = 0;
#endif
    GroupOfFrame<PointEncode> frames;
    GroupOfFrame<PointEncode> dynamicFrames;
    GroupOfFrame<PointEncode> staticFrames;
    GroupOfFrame<PointEncode> staticAddedFrames;
    GroupOfFrame<PointEncode> staticRemovedFrames;
    size_t                    groupOfFramesSize = parameter.groupOfFramesSize;
    size_t                    frameNumber       = parameter.inputDataset.getStartFrameNumber();
    const size_t              endFrameNumber    = parameter.inputDataset.getEndFrameNumber();

    while (frameNumber < endFrameNumber) {
      clock.start();
      reader->loadAll(frameNumber, groupOfFramesSize, frames, parameter.parallel);
      preProcessor.process(frames, nullptr, parameter.parallel);

      dynamicFrames.clear();
      staticFrames.clear();
      staticAddedFrames.clear();
      staticRemovedFrames.clear();
      for (const auto& frame : frames) {
        auto                  dynamicFrame       = jpcc::make_shared<Frame<PointEncode>>();
        auto                  staticAddedFrame   = jpcc::make_shared<Frame<PointEncode>>();
        auto                  staticRemovedFrame = jpcc::make_shared<Frame<PointEncode>>();
        FramePtr<PointEncode> staticFrame;

#if defined(NDEBUG)
        if (parameter.outputStatic) { staticFrame = jpcc::make_shared<Frame<PointEncode>>(); }
#else
        staticFrame = jpcc::make_shared<Frame<PointEncode>>();
#endif

        gmmSegmentation->segmentation(frame, dynamicFrame, staticFrame, staticAddedFrame, staticRemovedFrame);

#if !defined(NDEBUG)
        staticPointSize += staticAddedFrame->size();
        staticPointSize -= staticRemovedFrame->size();
        assert(staticPointSize == staticFrame->size());
#endif

        dynamicFrames.push_back(dynamicFrame);
        staticAddedFrames.push_back(staticAddedFrame);
        staticRemovedFrames.push_back(staticRemovedFrame);
        if (parameter.outputStatic) { staticFrames.push_back(staticFrame); }
      }

      savePly<PointEncode, PointOutput>(dynamicFrames, parameter.outputDataset.getFilePath(0), parameter.parallel);
      savePly<PointEncode, PointOutput>(staticAddedFrames, parameter.outputDataset.getFilePath(1), parameter.parallel);
      savePly<PointEncode, PointOutput>(staticRemovedFrames, parameter.outputDataset.getFilePath(2),
                                        parameter.parallel);
      if (parameter.outputStatic) {
        savePly<PointEncode, PointOutput>(staticFrames, parameter.outputDataset.getFilePath(3), parameter.parallel);
      }
      clock.stop();

      frameNumber += groupOfFramesSize;
    }
  }
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