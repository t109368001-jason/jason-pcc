#include <chrono>
#include <iostream>
#include <vector>

#include <boost/range/counting_range.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/range/algorithm_ext.hpp>

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

  auto dynamicEncoder = JPCCEncoderAdapter::build<PointEncode>(parameter.jpccEncoderDynamic);
  auto staticEncoder  = JPCCEncoderAdapter::build<PointEncode>(parameter.jpccEncoderStatic);
  auto dynamicDecoder = JPCCDecoderAdapter::build<PointEncode>(parameter.jpccDecoderDynamic);
  auto staticDecoder  = JPCCDecoderAdapter::build<PointEncode>(parameter.jpccDecoderStatic);

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

  GroupOfFrame<PointEncode>        frames;
  GroupOfFrame<PointEncode>        dynamicFrames;
  GroupOfFrame<PointEncode>        staticFrames;
  GroupOfFrame<PointEncode>        staticAddedFrames;
  GroupOfFrame<PointEncode>        staticRemovedFrames;
  vector<JPCCContext<PointEncode>> dynamicContexts;
  vector<JPCCContext<PointEncode>> staticContexts;
  GroupOfFrame<PointEncode>        reconstructFrames;
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
    metric.addPoints<PointEncode>("Raw", frames, true);

    {  // preprocess
      ScopeStopwatch clock = metric.start("PreProcess", frameNumber);
      preProcessor.process(frames, nullptr, parameter.parallel);
    }
    metric.addPoints<PointEncode>("PreProcessed", frames, true);

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
      JPCCContext<PointEncode> dynamicContext;
      dynamicContext.pclFrame = dynamicFrame;
      JPCCContext<PointEncode> staticContext;
      staticContext.pclFrame = staticFrame;
      dynamicContexts.push_back(dynamicContext);
      staticContexts.push_back(staticContext);
    }

    // encode
    for (auto& dynamicContext : dynamicContexts) {
      dynamicEncoder->convertFromPCL(dynamicContext);
      dynamicEncoder->encode(dynamicContext);
    }
    for (auto& staticContext : staticContexts) {
      dynamicEncoder->convertFromPCL(staticContext);
      dynamicEncoder->encode(staticContext);
    }
    // save
    if (parameter.jpccEncoderDynamic.backendType != CoderBackendType::NONE) {
      for (auto& dynamicContext : dynamicContexts) {
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
      metric.addPoints<PointEncode>("Dynamic", dynamicFrames, true);
    }
    if (parameter.jpccEncoderStatic.backendType != CoderBackendType::NONE) {
      for (auto& staticContext : staticContexts) {
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
      metric.addPoints<PointEncode>("StaticAdded", staticAddedFrames, true);
      metric.addPoints<PointEncode>("StaticRemoved", staticRemovedFrames, true);
    }

    for (auto& frame : frames) {
      auto reconstructFrame = jpcc::make_shared<Frame<PointEncode>>();
      reconstructFrames.push_back(reconstructFrame);
    }

    // decode
    if (parameter.jpccDecoderDynamic.backendType != CoderBackendType::NONE) {
      for (size_t i = 0; i < dynamicContexts.size(); i++) {
        dynamicDecoder->decode(dynamicContexts.at(i));
        dynamicDecoder->convertToPCL(dynamicContexts.at(i));
        pcl::copyPointCloud(*dynamicContexts.at(i).reconstructPclFrame, *reconstructFrames.at(i));
      }
    } else {
      if (!parameter.parallel) {
        for (size_t i = 0; i < frames.size(); i++) {
          pcl::copyPointCloud(*dynamicFrames.at(i), *reconstructFrames.at(i));
        }
      } else {
        const auto range = boost::counting_range<size_t>(0, dynamicFrames.size());
        std::for_each(std::execution::par, range.begin(), range.end(),
                      [&](const size_t& i) {  //
                        pcl::copyPointCloud(*dynamicFrames.at(i), *reconstructFrames.at(i));
                      });
      }
    }
    if (parameter.jpccDecoderStatic.backendType != CoderBackendType::NONE) {
      for (size_t i = 0; i < staticContexts.size(); i++) {
        staticDecoder->decode(staticContexts.at(i));
        staticDecoder->convertToPCL(staticContexts.at(i));
        pcl::copyPointCloud(*staticContexts.at(i).reconstructPclFrame, *reconstructFrames.at(i));
      }
    } else {
      if (!parameter.parallel) {
        for (size_t i = 0; i < frames.size(); i++) {
          pcl::copyPointCloud(*staticFrames.at(i), *reconstructFrames.at(i));
        }
      } else {
        const auto range = boost::counting_range<size_t>(0, staticFrames.size());
        std::for_each(std::execution::par, range.begin(), range.end(),
                      [&](const size_t& i) {  //
                        pcl::copyPointCloud(*staticFrames.at(i), *reconstructFrames.at(i));
                      });
      }
    }

    GroupOfFrame<PointMetric> framesWithNormal = normalEstimation.computeAll(frames, parameter.parallel);
    GroupOfFrame<PointMetric> reconstructFramesWithNormal =
        normalEstimation.computeAll(reconstructFrames, parameter.parallel);
    metric.addPSNR<PointMetric, PointMetric>("A2B", framesWithNormal, reconstructFramesWithNormal, parameter.parallel);
    metric.addPSNR<PointMetric, PointMetric>("B2A", reconstructFramesWithNormal, framesWithNormal, parameter.parallel);

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