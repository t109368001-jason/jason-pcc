#include <chrono>
#include <iostream>
#include <vector>

#include <jpcc/common/ParameterParser.h>
#include <jpcc/io/PlyIO.h>
#include <jpcc/io/Reader.h>
#include <jpcc/metric/JPCCMetric.h>
#include <jpcc/process/PreProcessor.h>
#include <jpcc/segmentation/JPCCSegmentationAdapter.h>

#include "AppParameter.h"

using namespace std;
using namespace std::chrono;
using namespace jpcc;
using namespace jpcc::io;
using namespace jpcc::metric;
using namespace jpcc::process;
using namespace jpcc::segmentation;

using PointEncode = pcl::PointXYZI;
using PointOutput = pcl::PointXYZ;

void encode(const AppParameter& parameter, JPCCMetric& metric) {
  DatasetReader<PointEncode>::Ptr    reader = newReader<PointEncode>(parameter.inputReader, parameter.inputDataset);
  PreProcessor<PointEncode>          preProcessor(parameter.preProcess);
  JPCCSegmentation<PointEncode>::Ptr gmmSegmentation = JPCCSegmentationAdapter::build<PointEncode>(
      parameter.jpccGmmSegmentation, (int)parameter.inputDataset.getStartFrameNumber());

  {  // build gaussian mixture model
    GroupOfFrame<PointEncode> frames;
    size_t                    groupOfFramesSize = parameter.groupOfFramesSize;
    size_t                    frameNumber       = parameter.inputDataset.getStartFrameNumber();
    const size_t              endFrameNumber    = parameter.inputDataset.getEndFrameNumber();
    while (!gmmSegmentation->isBuilt() && frameNumber < endFrameNumber) {
      {
        auto clock = metric.start("Load", frameNumber);
        reader->loadAll(frameNumber, groupOfFramesSize, frames, parameter.parallel);
      }
      {
        auto clock = metric.start("PreProcess", frameNumber);
        preProcessor.process(frames, nullptr, parameter.parallel);
      }
      for (const auto& frame : frames) {
        auto clock = metric.start("Build", frame->header.seq);
        gmmSegmentation->appendTrainSamples(frame);
      }

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
    {
      auto clock = metric.start("Load", frameNumber);
      reader->loadAll(frameNumber, groupOfFramesSize, frames, parameter.parallel);
    }
    metric.addPoints<PointEncode>("Raw", frames);

    {
      auto clock = metric.start("PreProcess", frameNumber);
      preProcessor.process(frames, nullptr, parameter.parallel);
    }
    metric.addPoints<PointEncode>("PreProcessed", frames);

    // TODO extract JPCCEncoder
    dynamicFrames.clear();
    staticAddedFrames.clear();
    staticRemovedFrames.clear();
    for (const auto& frame : frames) {
      auto dynamicFrame       = jpcc::make_shared<Frame<PointEncode>>();
      auto staticAddedFrame   = jpcc::make_shared<Frame<PointEncode>>();
      auto staticRemovedFrame = jpcc::make_shared<Frame<PointEncode>>();

#if defined(NDEBUG)
      {
        auto clock = metric.start("Encode", frame->header.seq);
        gmmSegmentation->segmentation(frame, dynamicFrame, nullptr, staticAddedFrame, staticRemovedFrame);
      }

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
    {
      auto clock = metric.start("Save", frameNumber);
      // TODO extract JPCCWriter
      savePly<PointEncode, PointOutput>(dynamicFrames, parameter.outputDataset.getFilePath(0), parameter.parallel);
      savePly<PointEncode, PointOutput>(staticAddedFrames, parameter.outputDataset.getFilePath(1), parameter.parallel);
      savePly<PointEncode, PointOutput>(staticRemovedFrames, parameter.outputDataset.getFilePath(2),
                                        parameter.parallel);
    }

    metric.addPoints<PointEncode>("Dynamic", dynamicFrames);
    metric.addPoints<PointEncode>("StaticAdded", staticAddedFrames);
    metric.addPoints<PointEncode>("StaticRemoved", staticRemovedFrames);

    frameNumber += groupOfFramesSize;
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
    JPCCMetric metric(parameter.metricParameter);

    {
      auto clock = metric.start("Wall", parameter.inputDataset.getStartFrameNumber());
      encode(parameter, metric);
    }

    metric.writeAndShow();
  } catch (exception& e) { cerr << e.what() << endl; }

  cout << "JPCC App Encoder End" << endl;
  return 0;
}