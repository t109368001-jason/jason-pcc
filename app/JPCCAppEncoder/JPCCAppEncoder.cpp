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

  std::ofstream dynamicOfs("./bin/output-dynamic.bin", std::ios::binary);
  std::ofstream staticOfs("./bin/output-static.bin", std::ios::binary);

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
    for (const auto& frame : frames) { metric.addBytes("Raw", frame->header.seq, frame->size() * sizeof(float) * 3); }

    {  // preprocess
      ScopeStopwatch clock = metric.start("PreProcess", frameNumber);
      preProcessor.process(frames, nullptr, parameter.parallel);
    }
    metric.addPoints<PointEncode>("PreProcessed", frames);
    for (const auto& frame : frames) {
      metric.addBytes("PreProcessed", frame->header.seq, frame->size() * sizeof(float) * 3);
    }

    // segmentation
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

    // encode
    if (dynamicEncoder) {
      for (auto& dynamicContext : dynamicContexts) {
        dynamicEncoder->convertFromPCL(dynamicContext);
        dynamicEncoder->encode(dynamicContext);
        {  // save
          ScopeStopwatch clock = metric.start("Save", frameNumber);
          dynamicOfs.write(dynamicContext.encodedBytes.data(), dynamicContext.encodedBytes.size());
        }
        metric.addPoints<PointEncode>("Dynamic", dynamicContext.pclFrame);
        metric.addBytes("Dynamic", dynamicContext.pclFrame->header.seq, dynamicContext.encodedBytes.size());
      }
    } else {
      {  // save
        ScopeStopwatch clock = metric.start("Save", frameNumber);
        // TODO extract JPCCWriter
        savePly<PointEncode, PointOutput>(dynamicFrames, parameter.outputDataset.getFilePath(0), parameter.parallel);
      }
      metric.addPoints<PointEncode>("Dynamic", dynamicFrames);
      for (const auto& frame : dynamicFrames) {
        metric.addBytes("Dynamic", frame->header.seq, frame->size() * sizeof(float) * 3);
      }
    }
    if (staticEncoder) {
      for (auto& staticContext : staticContexts) {
        dynamicEncoder->convertFromPCL(staticContext);
        dynamicEncoder->encode(staticContext);
        {  // save
          ScopeStopwatch clock = metric.start("Save", frameNumber);
          staticOfs.write(staticContext.encodedBytes.data(), staticContext.encodedBytes.size());
        }
        metric.addPoints<PointEncode>("Static", staticContext.pclFrame);
        metric.addBytes("Static", staticContext.pclFrame->header.seq, staticContext.encodedBytes.size());
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
      for (const auto& frame : staticAddedFrames) {
        metric.addBytes("StaticAdded", frame->header.seq, frame->size() * sizeof(float) * 3);
      }
      metric.addPoints<PointEncode>("StaticRemoved", staticRemovedFrames);
      for (const auto& frame : staticRemovedFrames) {
        metric.addBytes("StaticRemoved", frame->header.seq, frame->size() * sizeof(float) * 3);
      }
    }

    for (auto& frame : frames) {
      auto reconstructFrame = jpcc::make_shared<Frame<PointEncode>>();
      reconstructFrames.push_back(reconstructFrame);
    }

    // decode
    if (dynamicDecoder) {
      for (size_t i = 0; i < dynamicContexts.size(); i++) {
        dynamicDecoder->decode(dynamicContexts.at(i));
        dynamicDecoder->convertToPCL(dynamicContexts.at(i));
        pcl::copyPointCloud(*dynamicContexts.at(i).reconstructPclFrame, *reconstructFrames.at(i));
      }
    } else {
      for (size_t i = 0; i < frames.size(); i++) {
        pcl::copyPointCloud(*dynamicFrames.at(i), *reconstructFrames.at(i));
      }
    }
    if (staticDecoder) {
      for (size_t i = 0; i < staticContexts.size(); i++) {
        staticDecoder->decode(staticContexts.at(i));
        staticDecoder->convertToPCL(staticContexts.at(i));
        pcl::copyPointCloud(*staticContexts.at(i).reconstructPclFrame, *reconstructFrames.at(i));
      }
    } else {
      for (size_t i = 0; i < frames.size(); i++) { pcl::copyPointCloud(*staticFrames.at(i), *reconstructFrames.at(i)); }
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