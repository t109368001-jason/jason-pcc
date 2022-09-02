#include <chrono>
#include <iostream>
#include <vector>

#include <jpcc/common/ParameterParser.h>
#include <jpcc/coder/JPCCEncoderAdapter.h>
#include <jpcc/coder/JPCCDecoderAdapter.h>
#include <jpcc/io/PlyIO.h>
#include <jpcc/io/Reader.h>
#include <jpcc/metric/JPCCMetric.h>
#include <jpcc/process/JPCCNormalEstimation.h>
#include <jpcc/process/PreProcessor.h>
#include <jpcc/segmentation/JPCCSegmentationAdapter.h>

#include "AppParameter.h"

using namespace std;
using namespace std::chrono;
using namespace jpcc;
using namespace jpcc::coder;
using namespace jpcc::io;
using namespace jpcc::metric;
using namespace jpcc::octree;
using namespace jpcc::process;
using namespace jpcc::segmentation;

using PointEncode = pcl::PointXYZI;
using PointOutput = pcl::PointXYZ;
using PointMetric = pcl::PointXYZINormal;

void encode(const AppParameter& parameter, JPCCMetric& metric) {
  DatasetReader<PointEncode>::Ptr    reader = newReader<PointEncode>(parameter.inputReader, parameter.inputDataset);
  PreProcessor<PointEncode>          preProcessor(parameter.preProcess);
  JPCCSegmentation<PointEncode>::Ptr gmmSegmentation = JPCCSegmentationAdapter::build<PointEncode>(
      parameter.jpccGmmSegmentation, (int)parameter.inputDataset.getStartFrameNumber());
  JPCCNormalEstimation<PointEncode, PointMetric> normalEstimation(parameter.normalEstimation);
  JPCCEncoder<PointEncode>::Ptr                  dynamicEncoder;
  JPCCEncoder<PointEncode>::Ptr                  staticEncoder;
  JPCCDecoder<PointEncode>::Ptr                  dynamicDecoder;
  JPCCDecoder<PointEncode>::Ptr                  staticDecoder;
  if (parameter.jpccEncoderDynamic.backendType != CoderBackendType::NONE) {
    dynamicEncoder = JPCCEncoderAdapter::build<PointEncode>(parameter.jpccEncoderDynamic);
  }
  if (parameter.jpccEncoderStatic.backendType != CoderBackendType::NONE) {
    staticEncoder = JPCCEncoderAdapter::build<PointEncode>(parameter.jpccEncoderStatic);
  }
  if (parameter.jpccDecoderDynamic.backendType != CoderBackendType::NONE) {
    dynamicDecoder = JPCCDecoderAdapter::build<PointEncode>(parameter.jpccDecoderDynamic);
  }
  if (parameter.jpccDecoderStatic.backendType != CoderBackendType::NONE) {
    staticDecoder = JPCCDecoderAdapter::build<PointEncode>(parameter.jpccDecoderStatic);
  }

  {  // build gaussian mixture model
    GroupOfFrame<PointEncode> frames;
    size_t                    groupOfFramesSize = parameter.groupOfFramesSize;
    size_t                    frameNumber       = parameter.inputDataset.getStartFrameNumber();
    const size_t              endFrameNumber    = parameter.inputDataset.getEndFrameNumber();
    while (!gmmSegmentation->isBuilt() && frameNumber < endFrameNumber) {
      {
        ScopeStopwatch clock = metric.start("Load", frameNumber);
        reader->loadAll(frameNumber, groupOfFramesSize, frames, parameter.parallel);
      }
      {
        ScopeStopwatch clock = metric.start("PreProcess", frameNumber);
        preProcessor.process(frames, nullptr, parameter.parallel);
      }
      for (const auto& frame : frames) {
        ScopeStopwatch clock = metric.start("Build", frame->header.seq);
        gmmSegmentation->appendTrainSamples(frame);
      }

      frameNumber += groupOfFramesSize;
    }
  }

  size_t       groupOfFramesSize = parameter.groupOfFramesSize;
  size_t       frameNumber       = parameter.inputDataset.getStartFrameNumber();
  const size_t endFrameNumber    = parameter.inputDataset.getEndFrameNumber();

  GroupOfFrame<PointEncode>             frames;
  GroupOfFrame<PointEncode>             dynamicFrames;
  GroupOfFrame<PointEncode>             staticFrames;
  GroupOfFrame<PointEncode>             staticAddedFrames;
  GroupOfFrame<PointEncode>             staticRemovedFrames;
  vector<JPCCCoderContext<PointEncode>> dynamicContexts;
  vector<JPCCCoderContext<PointEncode>> staticContexts;
  GroupOfFrame<PointEncode>             reconstructFrames;
  while (frameNumber < endFrameNumber) {
    {  // clear
      frames.clear();
      dynamicFrames.clear();
      staticAddedFrames.clear();
      staticRemovedFrames.clear();
      dynamicContexts.clear();
      staticContexts.clear();
      reconstructFrames.clear();
    }
    {  // load
      ScopeStopwatch clock = metric.start("Load", frameNumber);
      reader->loadAll(frameNumber, groupOfFramesSize, frames, parameter.parallel);
    }
    metric.addPoints<PointEncode>("Raw", frames);

    {  // preprocess
      ScopeStopwatch clock = metric.start("PreProcess", frameNumber);
      preProcessor.process(frames, nullptr, parameter.parallel);
    }
    metric.addPoints<PointEncode>("PreProcessed", frames);

    // TODO extract JPCCEncoder
    // encode
    for (const auto& frame : frames) {
      auto dynamicFrame       = jpcc::make_shared<Frame<PointEncode>>();
      auto staticFrame        = jpcc::make_shared<Frame<PointEncode>>();
      auto staticAddedFrame   = jpcc::make_shared<Frame<PointEncode>>();
      auto staticRemovedFrame = jpcc::make_shared<Frame<PointEncode>>();

      {
        ScopeStopwatch clock = metric.start("Encode", frame->header.seq);
        gmmSegmentation->segmentation(frame, dynamicFrame, staticFrame, staticAddedFrame, staticRemovedFrame);
      }

      dynamicFrames.push_back(dynamicFrame);
      staticFrames.push_back(staticFrame);
      staticAddedFrames.push_back(staticAddedFrame);
      staticRemovedFrames.push_back(staticRemovedFrame);
      JPCCCoderContext<PointEncode> dynamicContext;
      dynamicContext.pclFrame = dynamicFrame;
      JPCCCoderContext<PointEncode> staticContext;
      staticContext.pclFrame = staticFrame;
      dynamicContexts.push_back(dynamicContext);
      staticContexts.push_back(staticContext);
    }
    if (dynamicEncoder) {
      for (auto& dynamicContext : dynamicContexts) {
        dynamicEncoder->convertFromPCL(dynamicContext);
        dynamicEncoder->encode(dynamicContext);
        {  // save
          ScopeStopwatch clock = metric.start("Save", frameNumber);
          // TODO save
        }
      }
    } else {
      {  // save
        ScopeStopwatch clock = metric.start("Save", frameNumber);
        // TODO extract JPCCWriter
        savePly<PointEncode, PointOutput>(dynamicFrames, parameter.outputDataset.getFilePath(0), parameter.parallel);
      }
      metric.addPoints<PointEncode>("Dynamic", dynamicFrames);
    }
    if (staticEncoder) {
      for (auto& staticContext : staticContexts) {
        dynamicEncoder->convertFromPCL(staticContext);
        dynamicEncoder->encode(staticContext);
        {  // save
          ScopeStopwatch clock = metric.start("Save", frameNumber);
          // TODO save
        }
      }
    } else {
      {  // save
        ScopeStopwatch clock = metric.start("Save", frameNumber);
        // TODO extract JPCCWriter
        savePly<PointEncode, PointOutput>(staticAddedFrames, parameter.outputDataset.getFilePath(1),
                                          parameter.parallel);
        savePly<PointEncode, PointOutput>(staticRemovedFrames, parameter.outputDataset.getFilePath(2),
                                          parameter.parallel);
      }
      metric.addPoints<PointEncode>("StaticAdded", staticAddedFrames);
      metric.addPoints<PointEncode>("StaticRemoved", staticRemovedFrames);
    }

    // decode
    if (dynamicDecoder) {
    } else {
      for (size_t i = 0; i < frames.size(); i++) {
        auto reconstructFrame = jpcc::make_shared<Frame<PointEncode>>();
        pcl::copyPointCloud(*dynamicFrames.at(i), *reconstructFrame);
        pcl::copyPointCloud(*staticFrames.at(i), *reconstructFrame);
        reconstructFrames.push_back(reconstructFrame);
      }
    }

    for (size_t i = 0; i < frames.size(); i++) {
      FramePtr<PointMetric> frameWithNormal            = normalEstimation.compute(frames.at(i));
      FramePtr<PointMetric> reconstructFrameWithNormal = normalEstimation.compute(reconstructFrames.at(i));
      metric.addPSNR<PointMetric, PointMetric>("A2B", frameWithNormal, reconstructFrameWithNormal);
      metric.addPSNR<PointMetric, PointMetric>("B2A", reconstructFrameWithNormal, frameWithNormal);
    }

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
      ScopeStopwatch clock = metric.start("Wall", parameter.inputDataset.getStartFrameNumber());
      encode(parameter, metric);
    }

    metric.writeAndShow();
  } catch (exception& e) { cerr << e.what() << endl; }

  cout << "JPCC App Encoder End" << endl;
  return 0;
}