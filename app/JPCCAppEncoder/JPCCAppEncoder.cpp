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

  JPCCEncoder<Point>::Ptr dynamicEncoder       = JPCCEncoderAdapter::build<Point>(parameter.jpccEncoderDynamic);
  JPCCEncoder<Point>::Ptr staticEncoder        = JPCCEncoderAdapter::build<Point>(parameter.jpccEncoderStatic);
  JPCCEncoder<Point>::Ptr staticAddedEncoder   = JPCCEncoderAdapter::build<Point>(parameter.jpccEncoderStatic);
  JPCCEncoder<Point>::Ptr staticRemovedEncoder = JPCCEncoderAdapter::build<Point>(parameter.jpccEncoderStatic);
  JPCCDecoder<Point>::Ptr dynamicDecoder       = JPCCDecoderAdapter::build<Point>(parameter.jpccDecoderDynamic);
  JPCCDecoder<Point>::Ptr staticDecoder        = JPCCDecoderAdapter::build<Point>(parameter.jpccDecoderStatic);
  JPCCDecoder<Point>::Ptr staticAddedDecoder   = JPCCDecoderAdapter::build<Point>(parameter.jpccDecoderStatic);
  JPCCDecoder<Point>::Ptr staticRemovedDecoder = JPCCDecoderAdapter::build<Point>(parameter.jpccDecoderStatic);

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

  std::ofstream dynamicOfs(parameter.compressedDynamicStreamPath, std::ios::binary);
  std::ofstream staticOfs(parameter.compressedStaticStreamPath, std::ios::binary);
  std::ofstream staticAddedOfs(parameter.compressedStaticAddedStreamPath, std::ios::binary);
  std::ofstream staticRemovedOfs(parameter.compressedStaticRemovedStreamPath, std::ios::binary);

  JPCCContext<pcl::PointXYZINormal> context(parameter.jpccGmmSegmentation.type,
                                            parameter.jpccGmmSegmentation.outputType);
  while (frameNumber < endFrameNumber) {
    size_t groupOfFramesSize = std::min(parameter.groupOfFramesSize, endFrameNumber - frameNumber);
    {  // clear
      context.clear();
      context.init(groupOfFramesSize);
    }
    {  // load
      ScopeStopwatch clock = metric.start("Load", frameNumber);
      reader->loadAll(frameNumber, groupOfFramesSize, context.getPclFrames(), parameter.parallel);
    }
    metric.addPoints<Point>("Raw", context.getPclFrames(), true);
    {  // preprocess
      ScopeStopwatch clock = metric.start("PreProcess", frameNumber);
      preProcessor.process(context.getPclFrames(), nullptr, parameter.parallel);
    }
    metric.addPoints<Point>("PreProcessed", context.getPclFrames(), true);
    {  // compute normal
      ScopeStopwatch clock = metric.start("ComputeNormal", frameNumber);
      normalEstimation.computeInPlaceAll(context.getPclFrames(), parameter.parallel);
    }
    {  // segmentation
      ScopeStopwatch clock = metric.start("Segmentation", frameNumber);
      gmmSegmentation->segmentation(context.getPclFrames(), context.getDynamicPclFrames(), context.getStaticPclFrames(),
                                    context.getStaticAddedPclFrames(), context.getStaticRemovedPclFrames(),
                                    parameter.parallel);
    }
    {  // convertFromPCL
      ScopeStopwatch clock = metric.start("ConvertFromPCL", frameNumber);
      dynamicEncoder->convertFromPCL(context.getDynamicPclFrames(), context.getDynamicFrames(), parameter.parallel);
      staticEncoder->convertFromPCL(context.getStaticPclFrames(), context.getStaticFrames(), parameter.parallel);
      staticAddedEncoder->convertFromPCL(context.getStaticAddedPclFrames(), context.getStaticAddedFrames(),
                                         parameter.parallel);
      staticRemovedEncoder->convertFromPCL(context.getStaticRemovedPclFrames(), context.getStaticRemovedFrames(),
                                           parameter.parallel);
    }
    {  // encode
      ScopeStopwatch clock = metric.start("Encode", frameNumber);
      dynamicEncoder->encode(context.getDynamicFrames(), context.getDynamicEncodedBytes(), parameter.parallel);
      staticEncoder->encode(context.getStaticFrames(), context.getStaticEncodedBytes(), parameter.parallel);
      staticAddedEncoder->encode(context.getStaticAddedFrames(), context.getStaticAddedEncodedBytes(),
                                 parameter.parallel);
      staticRemovedEncoder->encode(context.getStaticRemovedFrames(), context.getStaticRemovedEncodedBytes(),
                                   parameter.parallel);
    }
    metric.addPoints<Point>("Dynamic", context.getDynamicPclFrames());
    metric.addPoints<Point>("Static", context.getStaticPclFrames());
    metric.addPoints<Point>("StaticAdded", context.getStaticAddedPclFrames());
    metric.addPoints<Point>("StaticRemoved", context.getStaticRemovedPclFrames());
    metric.addBytes("Dynamic", frameNumber, context.getDynamicEncodedBytes().size());
    metric.addBytes("Static", frameNumber, context.getStaticEncodedBytes().size());
    metric.addBytes("StaticAdded", frameNumber, context.getStaticAddedEncodedBytes().size());
    metric.addBytes("StaticRemoved", frameNumber, context.getStaticRemovedEncodedBytes().size());

    // TODO extract JPCCWriter
    {  // save
      ScopeStopwatch clock = metric.start("Save", frameNumber);
      dynamicOfs.write(context.getDynamicEncodedBytes().data(),
                       (std::streamsize)context.getDynamicEncodedBytes().size());
      staticOfs.write(context.getStaticEncodedBytes().data(), (std::streamsize)context.getStaticEncodedBytes().size());
      staticAddedOfs.write(context.getStaticAddedEncodedBytes().data(),
                           (std::streamsize)context.getStaticAddedEncodedBytes().size());
      staticRemovedOfs.write(context.getStaticRemovedEncodedBytes().data(),
                             (std::streamsize)context.getStaticRemovedEncodedBytes().size());
    }
    {  // decode
      ScopeStopwatch clock = metric.start("Decode", frameNumber);

      std::istringstream dynamicIs(
          std::string(context.getDynamicEncodedBytes().begin(), context.getDynamicEncodedBytes().end()));
      std::istringstream staticIs(
          std::string(context.getStaticEncodedBytes().begin(), context.getStaticEncodedBytes().end()));
      std::istringstream staticAddedReconIs(
          std::string(context.getStaticAddedEncodedBytes().begin(), context.getStaticAddedEncodedBytes().end()));
      std::istringstream staticRemovedIs(
          std::string(context.getStaticRemovedEncodedBytes().begin(), context.getStaticRemovedEncodedBytes().end()));
      dynamicDecoder->decode(dynamicIs, context.getDynamicReconstructFrames(), parameter.parallel);
      staticDecoder->decode(staticIs, context.getStaticReconstructFrames(), parameter.parallel);
      staticAddedDecoder->decode(staticAddedReconIs, context.getStaticAddedReconstructFrames(), parameter.parallel);
      staticRemovedDecoder->decode(staticRemovedIs, context.getStaticRemovedReconstructFrames(), parameter.parallel);
    }
    {  // convertToPCL
      ScopeStopwatch clock = metric.start("ConvertToPCL", frameNumber);
      dynamicDecoder->convertToPCL(context.getDynamicReconstructFrames(), context.getDynamicReconstructPclFrames(),
                                   parameter.parallel);
      staticDecoder->convertToPCL(context.getStaticReconstructFrames(), context.getStaticReconstructPclFrames(),
                                  parameter.parallel);
      staticAddedDecoder->convertToPCL(context.getStaticAddedReconstructFrames(),
                                       context.getStaticAddedReconstructPclFrames(), parameter.parallel);
      staticRemovedDecoder->convertToPCL(context.getStaticRemovedReconstructFrames(),
                                         context.getStaticRemovedReconstructPclFrames(), parameter.parallel);
    }
    {  // copy normal to Reconstruct
      ScopeStopwatch clock = metric.start("CopyNormalToReconstruct", frameNumber);
      for (size_t i = 0; i < context.getDynamicReconstructPclFrames().size(); i++) {
        FramePtr<Point>& dynamicReconstructPclFrame = context.getDynamicReconstructPclFrames()[i];
        FramePtr<Point>& dynamicPclFrame            = context.getDynamicPclFrames()[i];
        for (size_t j = 0; j < dynamicReconstructPclFrame->points.size(); j++) {
          (*dynamicReconstructPclFrame)[j].normal_x  = (*dynamicPclFrame)[j].normal_x;
          (*dynamicReconstructPclFrame)[j].normal_y  = (*dynamicPclFrame)[j].normal_y;
          (*dynamicReconstructPclFrame)[j].normal_z  = (*dynamicPclFrame)[j].normal_z;
          (*dynamicReconstructPclFrame)[j].curvature = (*dynamicPclFrame)[j].curvature;
        }
      }
      for (size_t i = 0; i < context.getStaticReconstructPclFrames().size(); i++) {
        FramePtr<Point>& staticReconstructPclFrame = context.getStaticReconstructPclFrames()[i];
        FramePtr<Point>& staticPclFrames           = context.getStaticPclFrames()[i];
        for (size_t j = 0; j < staticReconstructPclFrame->points.size(); j++) {
          (*staticReconstructPclFrame)[j].normal_x  = (*staticPclFrames)[j].normal_x;
          (*staticReconstructPclFrame)[j].normal_y  = (*staticPclFrames)[j].normal_y;
          (*staticReconstructPclFrame)[j].normal_z  = (*staticPclFrames)[j].normal_z;
          (*staticReconstructPclFrame)[j].curvature = (*staticPclFrames)[j].curvature;
        }
      }
      for (size_t i = 0; i < context.getStaticAddedReconstructPclFrames().size(); i++) {
        FramePtr<Point>& staticAddedReconstructPclFrame = context.getStaticAddedReconstructPclFrames()[i];
        FramePtr<Point>& staticAddedPclFrame            = context.getStaticAddedPclFrames()[i];
        for (size_t j = 0; j < staticAddedReconstructPclFrame->points.size(); j++) {
          (*staticAddedReconstructPclFrame)[j].normal_x  = (*staticAddedPclFrame)[j].normal_x;
          (*staticAddedReconstructPclFrame)[j].normal_y  = (*staticAddedPclFrame)[j].normal_y;
          (*staticAddedReconstructPclFrame)[j].normal_z  = (*staticAddedPclFrame)[j].normal_z;
          (*staticAddedReconstructPclFrame)[j].curvature = (*staticAddedPclFrame)[j].curvature;
        }
      }
      for (size_t i = 0; i < context.getStaticRemovedReconstructPclFrames().size(); i++) {
        FramePtr<Point>& staticRemovedReconstructPclFrame = context.getStaticRemovedReconstructPclFrames()[i];
        FramePtr<Point>& staticRemovedPclFrame            = context.getStaticRemovedPclFrames()[i];
        for (size_t j = 0; j < staticRemovedReconstructPclFrame->points.size(); j++) {
          (*staticRemovedReconstructPclFrame)[j].normal_x  = (*staticRemovedPclFrame)[j].normal_x;
          (*staticRemovedReconstructPclFrame)[j].normal_y  = (*staticRemovedPclFrame)[j].normal_y;
          (*staticRemovedReconstructPclFrame)[j].normal_z  = (*staticRemovedPclFrame)[j].normal_z;
          (*staticRemovedReconstructPclFrame)[j].curvature = (*staticRemovedPclFrame)[j].curvature;
        }
      }
    }
    {  // combination
      ScopeStopwatch clock = metric.start("Combination", frameNumber);
      combination.combine(context.getDynamicReconstructPclFrames(), context.getStaticReconstructPclFrames(),
                          context.getStaticAddedReconstructPclFrames(), context.getStaticRemovedReconstructPclFrames(),
                          context.getStaticReconstructPclFrames(), context.getReconstructPclFrames(),
                          parameter.parallel);
    }
    {  // compute PSNR
      for (size_t i = 0; i < context.getPclFrames().size(); i++) {
        context.getReconstructPclFrames()[i]->header = context.getPclFrames()[i]->header;
      }
      ScopeStopwatch clock = metric.start("ComputePSNR", frameNumber);
      metric.addPSNR<Point, Point>("A2B", context.getPclFrames(), context.getReconstructPclFrames(),
                                   parameter.parallel);
      metric.addPSNR<Point, Point>("B2A", context.getReconstructPclFrames(), context.getPclFrames(),
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

  // Timers to count elapsed wall/user time
  JPCCMetric metric(parameter.metricParameter);

  {
    ScopeStopwatch clock = metric.start("Wall", parameter.inputDataset.getStartFrameNumber());
    encode(parameter, metric);
  }

  metric.writeAndShow();

  cout << "JPCC App Encoder End" << endl;
  return 0;
}