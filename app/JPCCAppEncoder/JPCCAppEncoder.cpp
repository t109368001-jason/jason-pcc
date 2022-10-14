#include <chrono>
#include <iostream>
#include <vector>

#include <pcl/kdtree/kdtree_flann.h>

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

  JPCCEncoderAdapter<Point> encoder(parameter.jpccEncoderDynamic, parameter.jpccEncoderStatic);
  JPCCDecoderAdapter<Point> decoder(parameter.jpccDecoderDynamic, parameter.jpccDecoderStatic);

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
      gmmSegmentation->segmentation(context, parameter.parallel);
    }
    {  // convertFromPCL
      ScopeStopwatch clock = metric.start("ConvertFromPCL", frameNumber);
      encoder.convertFromPCL(context, parameter.parallel);
    }
    {  // encode
      ScopeStopwatch clock = metric.start("Encode", frameNumber);
      encoder.encode(context, parameter.parallel);
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
      decoder.decode(context, groupOfFramesSize, parameter.parallel);
    }
    {  // convertToPCL
      ScopeStopwatch clock = metric.start("ConvertToPCL", frameNumber);
      decoder.convertToPCL(context, parameter.parallel);
    }
    {  // copy normal to Reconstruct
      ScopeStopwatch clock = metric.start("CopyNormalToReconstruct", frameNumber);
      metric.copyNormalToReconstruct(context, parameter.parallel);
    }
    {  // combination
      ScopeStopwatch clock = metric.start("Combination", frameNumber);
      combination.combine(context, parameter.parallel);
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
    if (parameter.jpccGmmSegmentation.type != jpcc::SegmentationType::NONE) {
      for (size_t i = 0; i < context.getPclFrames().size(); i++) {
        context.getDynamicReconstructPclFrames()[i]->header = context.getPclFrames()[i]->header;
      }
      // compute PSNR (Dynamic)
      ScopeStopwatch clock = metric.start("ComputePSNR(Dynamic)", frameNumber);
      metric.addPSNR<Point, Point>("A2B(Dynamic)", context.getPclFrames(), context.getDynamicReconstructPclFrames(),
                                   parameter.parallel);
      metric.addPSNR<Point, Point>("B2A(Dynamic)", context.getDynamicReconstructPclFrames(), context.getPclFrames(),
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