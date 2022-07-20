#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include <jpcc/common/ParameterParser.h>
#include <jpcc/io/PlyIO.h>
#include <jpcc/io/Reader.h>
#include <jpcc/process/PreProcessor.h>
#include <jpcc/segmentation/JPCCSegmentation.h>

#include "AppParameter.h"

using namespace std;
using namespace std::chrono;
using namespace pcc;
using namespace pcc::chrono;
using namespace jpcc;
using namespace jpcc::io;
using namespace jpcc::process;
using namespace jpcc::segmentation;

void encode(const AppParameter& parameter, StopwatchUserTime& clock) {
  DatasetReader::Ptr reader = newReader(parameter.inputReader, parameter.inputDataset);
  PreProcessor       preProcessor(parameter.preProcess);
  JPCCSegmentation   gmmSegmentation(parameter.jpccGmmSegmentation);

  {  // build gaussian mixture model
    GroupOfFrame frames;
    size_t       groupOfFramesSize = parameter.groupOfFramesSize;
    size_t       frameNumber       = parameter.inputDataset.getStartFrameNumber();
    const size_t endFrameNumber    = frameNumber + gmmSegmentation.getNTrain();
    while (frameNumber < endFrameNumber) {
      reader->loadAll(frameNumber, groupOfFramesSize, frames, parameter.parallel);
      preProcessor.process(frames, nullptr, parameter.parallel);

      clock.start();
      gmmSegmentation.appendTrainSamples(frames);
      clock.stop();

      frameNumber += groupOfFramesSize;
    }
    clock.start();
    gmmSegmentation.build();
    clock.stop();
  }

  {  // encode
    GroupOfFrame frames;
    GroupOfFrame dynamicFrames;
    GroupOfFrame staticFrames;
    GroupOfFrame staticAddedFrames;
    GroupOfFrame staticRemovedFrames;
    size_t       groupOfFramesSize = parameter.groupOfFramesSize;
    size_t       frameNumber       = parameter.inputDataset.getStartFrameNumber();
    const size_t endFrameNumber    = parameter.inputDataset.getEndFrameNumber();

    while (frameNumber < endFrameNumber) {
      clock.start();
      {  // load
        reader->loadAll(frameNumber, groupOfFramesSize, frames, parameter.parallel);
        preProcessor.process(frames, nullptr, parameter.parallel);
      }

      {  // encode
        // TODO extract JPCCEncoder
        dynamicFrames.clear();
        staticFrames.clear();
        staticAddedFrames.clear();
        staticRemovedFrames.clear();
        for (const auto& frame : frames) {
          auto     dynamicFrame = jpcc::make_shared<Frame>();
          FramePtr staticFrame;
          FramePtr staticAddedFrame;
          FramePtr staticRemovedFrame;
          switch (parameter.outputDataset.encodedType) {
            case EncodeType::DYNAMIC_STATIC: staticFrame = jpcc::make_shared<Frame>(); break;
            case EncodeType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED:
              staticAddedFrame   = jpcc::make_shared<Frame>();
              staticRemovedFrame = jpcc::make_shared<Frame>();
              break;
            default: break;
          }

          gmmSegmentation.segmentation(frame, dynamicFrame, staticFrame, staticAddedFrame, staticRemovedFrame);

          dynamicFrames.push_back(dynamicFrame);
          if (staticFrame) { staticFrames.push_back(staticFrame); }
          if (staticAddedFrame) { staticAddedFrames.push_back(staticAddedFrame); }
          if (staticRemovedFrame) { staticRemovedFrames.push_back(staticRemovedFrame); }
        }
      }
      {  // write
        // TODO extract JPCCWriter
        savePly(dynamicFrames, parameter.outputDataset.getFilePath(0), parameter.parallel);
        if (!staticFrames.empty()) {
          savePly(staticFrames, parameter.outputDataset.getFilePath(1), parameter.parallel);
        }
        if (!staticAddedFrames.empty()) {
          savePly(staticAddedFrames, parameter.outputDataset.getFilePath(1), parameter.parallel);
        }
        if (!staticRemovedFrames.empty()) {
          savePly(staticRemovedFrames, parameter.outputDataset.getFilePath(2), parameter.parallel);
        }
      }
      clock.stop();
      frameNumber += groupOfFramesSize;
    }
  }
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
    StopwatchUserTime       clockUser;

    clockWall.start();
    encode(parameter, clockUser);
    clockWall.stop();

    auto totalWall      = duration_cast<milliseconds>(clockWall.count()).count();
    auto totalUserSelf  = duration_cast<milliseconds>(clockUser.self.count()).count();
    auto totalUserChild = duration_cast<milliseconds>(clockUser.children.count()).count();
    cout << "Processing time (wall): " << (float)totalWall / 1000.0 << " s\n";
    cout << "Processing time (user.self): " << (float)totalUserSelf / 1000.0 << " s\n";
    cout << "Processing time (user.children): " << (float)totalUserChild / 1000.0 << " s\n";
    cout << "Peak memory: " << getPeakMemory() << " KB\n";
  } catch (exception& e) { cerr << e.what() << endl; }

  cout << "JPCC App Encoder End" << endl;
  return 0;
}