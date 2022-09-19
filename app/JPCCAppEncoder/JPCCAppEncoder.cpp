#include <chrono>
#include <iostream>
#include <vector>

#include <jpcc/common/JPCCContext.h>
#include <jpcc/common/ParameterParser.h>
#include <jpcc/coder/JPCCEncoderAdapter.h>
#include <jpcc/coder/JPCCDecoderAdapter.h>
#include <jpcc/io/PlyIO.h>
#include <jpcc/io/Reader.h>
#include <jpcc/metric/JPCCMetric.h>
#include <jpcc/process/JPCCNormalEstimation.h>
#include <jpcc/process/PreProcessor.h>
#include <jpcc/segmentation/JPCCSegmentationAdapter.h>
#include <jpcc/segmentation/JPCCCombination.h>

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
  JPCCCombination<PointEncode>                   combination(parameter.jpccGmmSegmentation.resolution);
  JPCCNormalEstimation<PointEncode, PointMetric> normalEstimation(parameter.normalEstimation);

  JPCCEncoder<PointEncode>::Ptr dynamicEncoder = JPCCEncoderAdapter::build<PointEncode>(parameter.jpccEncoderDynamic);
  JPCCEncoder<PointEncode>::Ptr staticEncoder  = JPCCEncoderAdapter::build<PointEncode>(parameter.jpccEncoderStatic);
  JPCCDecoder<PointEncode>::Ptr dynamicDecoder = JPCCDecoderAdapter::build<PointEncode>(parameter.jpccDecoderDynamic);
  JPCCDecoder<PointEncode>::Ptr staticDecoder  = JPCCDecoderAdapter::build<PointEncode>(parameter.jpccDecoderStatic);

  {  // build gaussian mixture model
    GroupOfFrame<PointEncode> frames;
    size_t                    groupOfFramesSize = parameter.groupOfFramesSize;
    size_t                    frameNumber       = parameter.inputDataset.getStartFrameNumber();
    const size_t              endFrameNumber    = parameter.inputDataset.getEndFrameNumber();
    while (!gmmSegmentation->isBuilt() && frameNumber < endFrameNumber) {
      {
        ScopeStopwatch clock = metric.start("Load", frameNumber);
        reader->loadAll(frameNumber, parameter.groupOfFramesSize, frames, parameter.parallel);
      }
      {
        ScopeStopwatch clock = metric.start("PreProcess", frameNumber);
        preProcessor.process(frames, nullptr, parameter.parallel);
      }
      for (const auto& frame : frames) {
        ScopeStopwatch clock = metric.start("Build", frame->header.seq);
        gmmSegmentation->appendTrainSamplesAndBuild(frame);
      }

      frameNumber += parameter.groupOfFramesSize;
    }
  }

  size_t       frameNumber    = parameter.inputDataset.getStartFrameNumber();
  const size_t endFrameNumber = parameter.inputDataset.getEndFrameNumber();

  std::ofstream dynamicOfs("./bin/output-dynamic.bin", std::ios::binary);
  std::ofstream staticOfs("./bin/output-static.bin", std::ios::binary);
  std::ofstream staticAddedOfs("./bin/output-static-added.bin", std::ios::binary);
  std::ofstream staticRemovedOfs("./bin/output-static-removed.bin", std::ios::binary);

  JPCCContext<PointEncode> context;
  context.segmentationOutputType = parameter.jpccGmmSegmentation.outputType;
  while (frameNumber < endFrameNumber) {
    size_t groupOfFramesSize = std::min(parameter.groupOfFramesSize, endFrameNumber - frameNumber);
    {  // clear
      context.clear();
      context.init(groupOfFramesSize);
    }
    {  // load
      ScopeStopwatch clock = metric.start("Load", frameNumber);
      reader->loadAll(frameNumber, groupOfFramesSize, context.pclFrames, parameter.parallel);
    }
    metric.addPoints<PointEncode>("Raw", context.pclFrames, true);

    {  // preprocess
      ScopeStopwatch clock = metric.start("PreProcess", frameNumber);
      preProcessor.process(context.pclFrames, nullptr, parameter.parallel);
    }
    metric.addPoints<PointEncode>("PreProcessed", context.pclFrames, true);

    // segmentation
    {
      ScopeStopwatch clock = metric.start("Segmentation", frameNumber);
      gmmSegmentation->segmentation(context.pclFrames, context.dynamicPclFrames, context.staticPclFrames,
                                    context.staticAddedPclFrames, context.staticRemovedPclFrames, parameter.parallel);
    }

    // convertFromPCL
    {
      ScopeStopwatch clock = metric.start("ConvertFromPCL", frameNumber);
      dynamicEncoder->convertFromPCL(context.dynamicPclFrames, context.dynamicFrames, parameter.parallel);
      staticEncoder->convertFromPCL(context.staticPclFrames, context.staticFrames, parameter.parallel);
      staticEncoder->convertFromPCL(context.staticAddedPclFrames, context.staticAddedFrames, parameter.parallel);
      staticEncoder->convertFromPCL(context.staticRemovedPclFrames, context.staticRemovedFrames, parameter.parallel);
    }

    // encode
    {
      ScopeStopwatch clock = metric.start("Encode", frameNumber);
      staticEncoder->encode(context.staticFrames, context.staticEncodedFramesBytes, parameter.parallel);
      dynamicEncoder->encode(context.dynamicFrames, context.dynamicEncodedFramesBytes, parameter.parallel);
      staticEncoder->encode(context.staticAddedFrames, context.staticAddedEncodedFramesBytes, parameter.parallel);
      staticEncoder->encode(context.staticRemovedFrames, context.staticRemovedEncodedFramesBytes, parameter.parallel);
    }

    // TODO extract JPCCWriter
    // save
    if (parameter.jpccEncoderDynamic.backendType != CoderBackendType::NONE) {
      for (size_t i = 0; i < context.dynamicEncodedFramesBytes.size(); i++) {
        {  // save
          ScopeStopwatch clock = metric.start("Save", frameNumber);
          dynamicOfs.write(context.dynamicEncodedFramesBytes.at(i).data(),
                           context.dynamicEncodedFramesBytes.at(i).size());
        }
        metric.addPoints<PointEncode>("Dynamic", context.dynamicPclFrames.at(i));
        metric.addBytes("Dynamic", context.pclFrames.at(i)->header.seq, context.dynamicEncodedFramesBytes.at(i).size());
      }
    } else {
      {  // save
        ScopeStopwatch clock = metric.start("Save", frameNumber);
        // TODO extract JPCCWriter
        savePly<PointEncode, PointOutput>(context.dynamicPclFrames, parameter.outputDataset.getFilePath(0),
                                          parameter.parallel);
      }
      metric.addPoints<PointEncode>("Dynamic", context.dynamicPclFrames, true);
    }
    if (parameter.jpccEncoderStatic.backendType != CoderBackendType::NONE) {
      for (size_t i = 0; i < context.staticEncodedFramesBytes.size(); i++) {
        {  // save
          ScopeStopwatch clock = metric.start("Save", frameNumber);
          staticOfs.write(context.staticEncodedFramesBytes.at(i).data(), context.staticEncodedFramesBytes.at(i).size());
        }
        metric.addPoints<PointEncode>("Static", context.staticPclFrames.at(i));
        metric.addBytes("Static", context.pclFrames.at(i)->header.seq, context.staticEncodedFramesBytes.at(i).size());
      }
      for (size_t i = 0; i < context.staticAddedEncodedFramesBytes.size(); i++) {
        {  // save
          ScopeStopwatch clock = metric.start("Save", frameNumber);
          staticAddedOfs.write(context.staticAddedEncodedFramesBytes.at(i).data(),
                               context.staticAddedEncodedFramesBytes.at(i).size());
        }
        metric.addPoints<PointEncode>("StaticAdded", context.staticAddedPclFrames.at(i));
        metric.addBytes("StaticAdded", context.pclFrames.at(i)->header.seq,
                        context.staticAddedEncodedFramesBytes.at(i).size());
      }
      for (size_t i = 0; i < context.staticRemovedEncodedFramesBytes.size(); i++) {
        {  // save
          ScopeStopwatch clock = metric.start("Save", frameNumber);
          staticRemovedOfs.write(context.staticRemovedEncodedFramesBytes.at(i).data(),
                                 context.staticRemovedEncodedFramesBytes.at(i).size());
        }
        metric.addPoints<PointEncode>("StaticRemoved", context.staticRemovedPclFrames.at(i));
        metric.addBytes("StaticRemoved", context.pclFrames.at(i)->header.seq,
                        context.staticRemovedEncodedFramesBytes.at(i).size());
      }
    } else {
      {  // save
        ScopeStopwatch clock = metric.start("Save", frameNumber);
        savePly<PointEncode, PointOutput>(context.staticPclFrames, parameter.outputDataset.getFilePath(0),
                                          parameter.parallel);
        savePly<PointEncode, PointOutput>(context.staticAddedPclFrames, parameter.outputDataset.getFilePath(0),
                                          parameter.parallel);
        savePly<PointEncode, PointOutput>(context.staticRemovedPclFrames, parameter.outputDataset.getFilePath(0),
                                          parameter.parallel);
      }
      metric.addPoints<PointEncode>("Static", context.staticPclFrames, true);
      metric.addPoints<PointEncode>("StaticAdded", context.staticAddedPclFrames, true);
      metric.addPoints<PointEncode>("StaticRemoved", context.staticRemovedPclFrames, true);
    }

    // decode
    {
      ScopeStopwatch clock = metric.start("Decode", frameNumber);
      if (parameter.jpccDecoderDynamic.backendType != CoderBackendType::NONE) {
        dynamicDecoder->decode(context.dynamicEncodedFramesBytes, context.dynamicReconstructFrames, parameter.parallel);
      } else {
        for (size_t i = 0; i < context.dynamicPclFrames.size(); i++) {
          context.dynamicReconstructFrames.at(i) = context.dynamicPclFrames.at(i);
        }
      }
      if (parameter.jpccDecoderStatic.backendType != CoderBackendType::NONE) {
        staticDecoder->decode(context.staticEncodedFramesBytes, context.staticReconstructFrames, parameter.parallel);
        staticDecoder->decode(context.staticAddedEncodedFramesBytes, context.staticAddedReconstructFrames,
                              parameter.parallel);
        staticDecoder->decode(context.staticRemovedEncodedFramesBytes, context.staticRemovedReconstructFrames,
                              parameter.parallel);
      } else {
        for (size_t i = 0; i < context.staticPclFrames.size(); i++) {
          context.staticReconstructFrames.at(i) = context.staticPclFrames.at(i);
        }
        for (size_t i = 0; i < context.staticAddedPclFrames.size(); i++) {
          context.staticAddedReconstructFrames.at(i) = context.staticAddedPclFrames.at(i);
        }
        for (size_t i = 0; i < context.staticRemovedPclFrames.size(); i++) {
          context.staticRemovedReconstructFrames.at(i) = context.staticRemovedPclFrames.at(i);
        }
      }
    }

    // convertToPCL
    {
      ScopeStopwatch clock = metric.start("ConvertToPCL", frameNumber);
      dynamicDecoder->convertToPCL(context.dynamicReconstructFrames, context.dynamicReconstructPclFrames,
                                   parameter.parallel);
      staticDecoder->convertToPCL(context.staticReconstructFrames, context.staticReconstructPclFrames,
                                  parameter.parallel);
      staticDecoder->convertToPCL(context.staticAddedReconstructFrames, context.staticAddedReconstructPclFrames,
                                  parameter.parallel);
      staticDecoder->convertToPCL(context.staticRemovedReconstructFrames, context.staticRemovedReconstructPclFrames,
                                  parameter.parallel);
    }

    // combination
    {
      ScopeStopwatch clock = metric.start("Combination", frameNumber);
      combination.combine(context.dynamicReconstructPclFrames, context.staticReconstructPclFrames,
                          context.staticAddedReconstructPclFrames, context.staticRemovedReconstructPclFrames,
                          context.staticReconstructPclFrames, context.reconstructPclFrames, parameter.parallel);
    }

    {
      for (size_t i = 0; i < context.pclFrames.size(); i++) {
        context.reconstructPclFrames.at(i)->header = context.pclFrames.at(i)->header;
      }
      ScopeStopwatch clock = metric.start("ComputePSNR", frameNumber);

      GroupOfFrame<PointMetric> framesWithNormal = normalEstimation.computeAll(context.pclFrames, parameter.parallel);
      GroupOfFrame<PointMetric> reconstructFramesWithNormal =
          normalEstimation.computeAll(context.reconstructPclFrames, parameter.parallel);
      metric.addPSNR<PointMetric, PointMetric>("A2B", framesWithNormal, reconstructFramesWithNormal,
                                               parameter.parallel);
      metric.addPSNR<PointMetric, PointMetric>("B2A", reconstructFramesWithNormal, framesWithNormal,
                                               parameter.parallel);
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