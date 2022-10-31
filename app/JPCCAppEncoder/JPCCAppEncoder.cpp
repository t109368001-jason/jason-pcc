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
  JPCCNormalEstimation<Point, Point> normalEstimation(parameter.normalEstimation);

  JPCCEncoderAdapter<Point> encoder(parameter.jpccEncoderDynamic, parameter.jpccEncoderStatic);

  JPCCDecoderAdapter<Point> decoder;
  JPCCCombination<Point>    combination;

  JPCCContext<pcl::PointXYZINormal> context(
      parameter.jpccGmmSegmentation.resolution, parameter.jpccGmmSegmentation.type,
      parameter.jpccGmmSegmentation.outputType, parameter.jpccEncoderDynamic.backendType,
      parameter.jpccEncoderStatic.backendType);

  std::ofstream ofs(parameter.compressedStreamPath, std::ios::binary);

  writeJPCCHeader(context.getHeader(), ofs);
  ofs.flush();
  std::ifstream ifs(parameter.compressedStreamPath, std::ios::binary);
  {
    JPCCHeader header{};
    readJPCCHeader(ifs, &header);
    THROW_IF_NOT(header == context.getHeader());
    decoder.set(header);
    combination.set(header);
  }

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
    metric.addBytes("Dynamic", frameNumber, context.getDynamicEncodedBytesVector());
    metric.addBytes("Static", frameNumber, context.getStaticEncodedBytesVector());
    metric.addBytes("StaticAdded", frameNumber, context.getStaticAddedEncodedBytesVector());
    metric.addBytes("StaticRemoved", frameNumber, context.getStaticRemovedEncodedBytesVector());

    // TODO extract JPCCWriter
    {  // save
      ifs.close();
      ifs.open(parameter.compressedStreamPath, std::ios::binary);
      ifs.seekg(0, std::ios::end);
      ScopeStopwatch clock = metric.start("Save", frameNumber);
      writeJPCCContext(context, ofs);
      ofs.flush();
    }
    {  // decode
      ScopeStopwatch clock = metric.start("Decode", frameNumber);
      decoder.decode(ifs, context, groupOfFramesSize, parameter.parallel);
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
      ScopeStopwatch clock = metric.start("ComputePSNR (Dynamic)", frameNumber);
      metric.addPSNR<Point, Point>("A2B (Dynamic)", context.getPclFrames(), context.getDynamicReconstructPclFrames(),
                                   parameter.parallel);
      metric.addPSNR<Point, Point>("B2A (Dynamic)", context.getDynamicReconstructPclFrames(), context.getPclFrames(),
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