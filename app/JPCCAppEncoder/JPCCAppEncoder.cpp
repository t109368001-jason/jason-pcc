#include <chrono>
#include <iostream>
#include <vector>

#include <jpcc/common/JPCCContext.h>
#include <jpcc/common/ParameterParser.h>
#include <jpcc/coder/JPCCEncoderAdapter.h>
#include <jpcc/coder/JPCCDecoderAdapter.h>
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

using Point = pcl::PointXYZINormal;

void encode(const AppParameter& parameter, JPCCMetric& metric) {
  DatasetReader<Point>::Ptr    reader = newReader<Point>(parameter.inputReader, parameter.inputDataset);
  PreProcessor<Point>          preProcessor(parameter.preProcess);
  JPCCSegmentation<Point>::Ptr gmmSegmentation = JPCCSegmentationAdapter::build<Point>(
      parameter.jpccGmmSegmentation, (int)parameter.inputDataset.getStartFrameNumber());
  JPCCCombination<Point>             combination(parameter.jpccGmmSegmentation.resolution);
  JPCCNormalEstimation<Point, Point> normalEstimation(parameter.normalEstimation);

  JPCCEncoder<Point>::Ptr dynamicEncoder = JPCCEncoderAdapter::build<Point>(parameter.jpccEncoderDynamic);
  JPCCEncoder<Point>::Ptr staticEncoder  = JPCCEncoderAdapter::build<Point>(parameter.jpccEncoderStatic);
  JPCCDecoder<Point>::Ptr dynamicDecoder = JPCCDecoderAdapter::build<Point>(parameter.jpccDecoderDynamic);
  JPCCDecoder<Point>::Ptr staticDecoder  = JPCCDecoderAdapter::build<Point>(parameter.jpccDecoderStatic);

  {  // build gaussian mixture model
    GroupOfFrame<Point> frames;
    size_t              groupOfFramesSize = parameter.groupOfFramesSize;
    size_t              frameNumber       = parameter.inputDataset.getStartFrameNumber();
    const size_t        endFrameNumber    = parameter.inputDataset.getEndFrameNumber();
    while (!gmmSegmentation->isBuilt() && frameNumber < endFrameNumber) {
      {  // load
        ScopeStopwatch clock = metric.start("Load", frameNumber);
        reader->loadAll(frameNumber, parameter.groupOfFramesSize, frames, parameter.parallel);
      }
      {  // preprocess
        ScopeStopwatch clock = metric.start("PreProcess", frameNumber);
        preProcessor.process(frames, nullptr, parameter.parallel);
      }
      {  // build
        ScopeStopwatch clock = metric.start("Build", frameNumber);
        gmmSegmentation->appendTrainSamplesAndBuild(frames, parameter.parallel);
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

  JPCCContext<pcl::PointXYZINormal> context;
  context.segmentationType       = parameter.jpccGmmSegmentation.type;
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
    metric.addPoints<Point>("Raw", context.pclFrames, true);
    {  // preprocess
      ScopeStopwatch clock = metric.start("PreProcess", frameNumber);
      preProcessor.process(context.pclFrames, nullptr, parameter.parallel);
    }
    metric.addPoints<Point>("PreProcessed", context.pclFrames, true);
    {  // compute normal
      ScopeStopwatch clock = metric.start("ComputeNormal", frameNumber);
      normalEstimation.computeInPlaceAll(context.pclFrames, parameter.parallel);
    }
    {  // segmentation
      ScopeStopwatch clock = metric.start("Segmentation", frameNumber);
      gmmSegmentation->segmentation(context.pclFrames, context.dynamicPclFrames, context.staticPclFrames,
                                    context.staticAddedPclFrames, context.staticRemovedPclFrames, parameter.parallel);
    }
    {  // convertFromPCL
      ScopeStopwatch clock = metric.start("ConvertFromPCL", frameNumber);
      dynamicEncoder->convertFromPCL(context.dynamicPclFrames, context.dynamicFrames, parameter.parallel);
      staticEncoder->convertFromPCL(context.staticPclFrames, context.staticFrames, parameter.parallel);
      staticEncoder->convertFromPCL(context.staticAddedPclFrames, context.staticAddedFrames, parameter.parallel);
      staticEncoder->convertFromPCL(context.staticRemovedPclFrames, context.staticRemovedFrames, parameter.parallel);
    }
    {  // encode
      ScopeStopwatch clock = metric.start("Encode", frameNumber);
      staticEncoder->encode(context.staticFrames, context.staticEncodedFramesBytes, parameter.parallel);
      dynamicEncoder->encode(context.dynamicFrames, context.dynamicEncodedFramesBytes, parameter.parallel);
      staticEncoder->encode(context.staticAddedFrames, context.staticAddedEncodedFramesBytes, parameter.parallel);
      staticEncoder->encode(context.staticRemovedFrames, context.staticRemovedEncodedFramesBytes, parameter.parallel);
    }
    metric.addPointsAndBytes<Point>("Dynamic", context.dynamicPclFrames, context.dynamicEncodedFramesBytes);
    metric.addPointsAndBytes<Point>("Dynamic", context.staticPclFrames, context.staticEncodedFramesBytes);
    metric.addPointsAndBytes<Point>("Dynamic", context.staticAddedPclFrames, context.staticAddedEncodedFramesBytes);
    metric.addPointsAndBytes<Point>("Dynamic", context.staticRemovedPclFrames, context.staticRemovedEncodedFramesBytes);

    // TODO extract JPCCWriter
    {  // save
      ScopeStopwatch clock = metric.start("Save", frameNumber);
      for (auto& dynamicEncodedBytes : context.dynamicEncodedFramesBytes) {
        dynamicOfs.write(dynamicEncodedBytes.data(), (std::streamsize)dynamicEncodedBytes.size());
      }
      for (auto& staticEncodedBytes : context.staticEncodedFramesBytes) {
        staticOfs.write(staticEncodedBytes.data(), (std::streamsize)staticEncodedBytes.size());
      }
      for (auto& staticAddedEncodedBytes : context.staticAddedEncodedFramesBytes) {
        staticAddedOfs.write(staticAddedEncodedBytes.data(), (std::streamsize)staticAddedEncodedBytes.size());
      }
      for (auto& staticRemovedEncodedBytes : context.staticRemovedEncodedFramesBytes) {
        staticRemovedOfs.write(staticRemovedEncodedBytes.data(), (std::streamsize)staticRemovedEncodedBytes.size());
      }
    }
    {  // decode
      ScopeStopwatch clock = metric.start("Decode", frameNumber);
      dynamicDecoder->decode(context.dynamicEncodedFramesBytes, context.dynamicReconstructFrames, parameter.parallel);
      staticDecoder->decode(context.staticEncodedFramesBytes, context.staticReconstructFrames, parameter.parallel);
      staticDecoder->decode(context.staticAddedEncodedFramesBytes, context.staticAddedReconstructFrames,
                            parameter.parallel);
      staticDecoder->decode(context.staticRemovedEncodedFramesBytes, context.staticRemovedReconstructFrames,
                            parameter.parallel);
    }
    {  // convertToPCL
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
    {  // copy normal to Reconstruct
      ScopeStopwatch clock = metric.start("CopyNormalToReconstruct", frameNumber);
      for (size_t i = 0; i < context.dynamicReconstructPclFrames.size(); i++) {
        FramePtr<Point>& staticReconstructPclFrame = context.dynamicReconstructPclFrames.at(i);
        FramePtr<Point>& staticPclFrames           = context.dynamicPclFrames.at(i);
        for (size_t j = 0; j < staticReconstructPclFrame->points.size(); j++) {
          staticReconstructPclFrame->at(j).normal_x  = staticPclFrames->at(j).normal_x;
          staticReconstructPclFrame->at(j).normal_y  = staticPclFrames->at(j).normal_y;
          staticReconstructPclFrame->at(j).normal_z  = staticPclFrames->at(j).normal_z;
          staticReconstructPclFrame->at(j).curvature = staticPclFrames->at(j).curvature;
        }
      }
      for (size_t i = 0; i < context.staticReconstructPclFrames.size(); i++) {
        FramePtr<Point>& staticReconstructPclFrame = context.staticReconstructPclFrames.at(i);
        FramePtr<Point>& staticPclFrames           = context.staticPclFrames.at(i);
        for (size_t j = 0; j < staticReconstructPclFrame->points.size(); j++) {
          staticReconstructPclFrame->at(j).normal_x  = staticPclFrames->at(j).normal_x;
          staticReconstructPclFrame->at(j).normal_y  = staticPclFrames->at(j).normal_y;
          staticReconstructPclFrame->at(j).normal_z  = staticPclFrames->at(j).normal_z;
          staticReconstructPclFrame->at(j).curvature = staticPclFrames->at(j).curvature;
        }
      }
      for (size_t i = 0; i < context.staticAddedReconstructPclFrames.size(); i++) {
        FramePtr<Point>& staticReconstructPclFrame = context.staticAddedReconstructPclFrames.at(i);
        FramePtr<Point>& staticPclFrames           = context.staticAddedPclFrames.at(i);
        for (size_t j = 0; j < staticReconstructPclFrame->points.size(); j++) {
          staticReconstructPclFrame->at(j).normal_x  = staticPclFrames->at(j).normal_x;
          staticReconstructPclFrame->at(j).normal_y  = staticPclFrames->at(j).normal_y;
          staticReconstructPclFrame->at(j).normal_z  = staticPclFrames->at(j).normal_z;
          staticReconstructPclFrame->at(j).curvature = staticPclFrames->at(j).curvature;
        }
      }
      for (size_t i = 0; i < context.staticRemovedReconstructPclFrames.size(); i++) {
        FramePtr<Point>& staticReconstructPclFrame = context.staticRemovedReconstructPclFrames.at(i);
        FramePtr<Point>& staticPclFrames           = context.staticRemovedPclFrames.at(i);
        for (size_t j = 0; j < staticReconstructPclFrame->points.size(); j++) {
          staticReconstructPclFrame->at(j).normal_x  = staticPclFrames->at(j).normal_x;
          staticReconstructPclFrame->at(j).normal_y  = staticPclFrames->at(j).normal_y;
          staticReconstructPclFrame->at(j).normal_z  = staticPclFrames->at(j).normal_z;
          staticReconstructPclFrame->at(j).curvature = staticPclFrames->at(j).curvature;
        }
      }
    }
    {  // combination
      ScopeStopwatch clock = metric.start("Combination", frameNumber);
      combination.combine(context.dynamicReconstructPclFrames, context.staticReconstructPclFrames,
                          context.staticAddedReconstructPclFrames, context.staticRemovedReconstructPclFrames,
                          context.staticReconstructPclFrames, context.reconstructPclFrames, parameter.parallel);
    }
    {  // compute PSNR
      for (size_t i = 0; i < context.pclFrames.size(); i++) {
        context.reconstructPclFrames.at(i)->header = context.pclFrames.at(i)->header;
      }
      ScopeStopwatch clock = metric.start("ComputePSNR", frameNumber);
      metric.addPSNR<Point, Point>("A2B", context.pclFrames, context.reconstructPclFrames, parameter.parallel);
      metric.addPSNR<Point, Point>("B2A", context.reconstructPclFrames, context.pclFrames, parameter.parallel);
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