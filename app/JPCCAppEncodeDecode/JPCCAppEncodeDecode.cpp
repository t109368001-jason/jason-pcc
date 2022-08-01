#include <chrono>
#include <iostream>
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

using PointEncode = pcl::PointXYZI;
using PointOutput = pcl::PointXYZ;

void encode(const AppParameter& parameter, StopwatchUserTime& clockEncode, StopwatchUserTime& clockDecode) {
  DatasetReader<PointEncode>::Ptr reader = newReader<PointEncode>(parameter.inputReader, parameter.inputDataset);
  PreProcessor<PointEncode>       preProcessor(parameter.preProcess);
  JPCCSegmentation<PointEncode>   gmmSegmentation(parameter.jpccGmmSegmentation);

  clockEncode.start();
  {  // build gaussian mixture model
    GroupOfFrame<PointEncode> frames;
    size_t                    groupOfFramesSize = parameter.groupOfFramesSize;
    size_t                    frameNumber       = parameter.inputDataset.getStartFrameNumber();
    const size_t              endFrameNumber    = frameNumber + gmmSegmentation.getNTrain();
    while (frameNumber < endFrameNumber) {
      reader->loadAll(frameNumber, groupOfFramesSize, frames, parameter.parallel);
      preProcessor.process(frames, nullptr, parameter.parallel);

      gmmSegmentation.appendTrainSamples(frames);

      frameNumber += groupOfFramesSize;
    }
    gmmSegmentation.build();
  }
  clockEncode.stop();

  {  // encode
    GroupOfFrame<PointEncode> frames;
    GroupOfFrame<PointEncode> dynamicFrames;
    GroupOfFrame<PointEncode> staticAddedFrames;
    GroupOfFrame<PointEncode> staticRemovedFrames;
    size_t                    groupOfFramesSize = parameter.groupOfFramesSize;
    size_t                    frameNumber       = parameter.inputDataset.getStartFrameNumber();
    const size_t              endFrameNumber    = parameter.inputDataset.getEndFrameNumber();

    while (frameNumber < endFrameNumber) {
      clockEncode.start();
      {  // load
        reader->loadAll(frameNumber, groupOfFramesSize, frames, parameter.parallel);
        preProcessor.process(frames, nullptr, parameter.parallel);
      }

      {  // encode
        // TODO extract JPCCEncoder
        dynamicFrames.clear();
        staticAddedFrames.clear();
        staticRemovedFrames.clear();
        for (const auto& frame : frames) {
          auto dynamicFrame       = jpcc::make_shared<Frame<PointEncode>>();
          auto staticAddedFrame   = jpcc::make_shared<Frame<PointEncode>>();
          auto staticRemovedFrame = jpcc::make_shared<Frame<PointEncode>>();

          gmmSegmentation.segmentation(frame, dynamicFrame, nullptr, staticAddedFrame, staticRemovedFrame);

          dynamicFrames.push_back(dynamicFrame);
          staticAddedFrames.push_back(staticAddedFrame);
          staticRemovedFrames.push_back(staticRemovedFrame);
        }
      }
      {  // write
        // TODO extract JPCCWriter
        savePly<PointEncode, PointOutput>(dynamicFrames, parameter.outputDataset.getFilePath(0), parameter.parallel);
        savePly<PointEncode, PointOutput>(staticAddedFrames, parameter.outputDataset.getFilePath(1),
                                          parameter.parallel);
        savePly<PointEncode, PointOutput>(staticRemovedFrames, parameter.outputDataset.getFilePath(2),
                                          parameter.parallel);
      }
      clockEncode.stop();

      clockDecode.start();
      {  // decode
         // TODO impl JPCCDecoder
      }
      clockDecode.stop();

      {  // compute metrics
         // TODO impl JPCCMetrics
      }

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
    StopwatchUserTime       clockUserEncode;
    StopwatchUserTime       clockUserDecode;

    clockWall.start();
    encode(parameter, clockUserEncode, clockUserDecode);
    clockWall.stop();

    auto totalWall            = duration_cast<milliseconds>(clockWall.count()).count();
    auto totalUserSelfEncode  = duration_cast<milliseconds>(clockUserEncode.self.count()).count();
    auto totalUserChildEncode = duration_cast<milliseconds>(clockUserEncode.children.count()).count();
    auto totalUserSelfDecode  = duration_cast<milliseconds>(clockUserDecode.self.count()).count();
    auto totalUserChildDecode = duration_cast<milliseconds>(clockUserDecode.children.count()).count();
    cout << "Processing time (wall): " << (float)totalWall / 1000.0 << " s\n";
    cout << "Processing time (encode.user.self): " << (float)totalUserSelfEncode / 1000.0 << " s\n";
    cout << "Processing time (encode.user.children): " << (float)totalUserChildEncode / 1000.0 << " s\n";
    cout << "Processing time (decode.user.self): " << (float)totalUserSelfDecode / 1000.0 << " s\n";
    cout << "Processing time (decode.user.children): " << (float)totalUserChildDecode / 1000.0 << " s\n";
    cout << "Peak memory: " << getPeakMemory() << " KB\n";
  } catch (exception& e) { cerr << e.what() << endl; }

  cout << "JPCC App Encoder End" << endl;
  return 0;
}