#include <chrono>
#include <fstream>
#include <iostream>
#include <vector>

#include <jpcc/common/JPCCContext.h>
#include <jpcc/common/ParameterParser.h>
#include <jpcc/decoder/JPCCDecoderAdapter.h>
#include <jpcc/encoder/JPCCEncoderAdapter.h>
#include <jpcc/io/Reader.h>
#include <jpcc/metric/JPCCMetric.h>
#include <jpcc/process/JPCCNormalEstimation.h>
#include <jpcc/process/PreProcessor.h>
#include <jpcc/segmentation/JPCCSegmentationAdapter.h>
#include <jpcc/combination/JPCCCombination.h>

#include "AppParameter.h"

using namespace std;
using namespace std::chrono;
using namespace jpcc;
using namespace jpcc::decoder;
using namespace jpcc::encoder;
using namespace jpcc::io;
using namespace jpcc::metric;
using namespace jpcc::octree;
using namespace jpcc::process;
using namespace jpcc::segmentation;
using namespace jpcc::combination;

void encode(const AppParameter& parameter, JPCCMetric& metric) {
  DatasetReader::Ptr    reader = newReader(parameter.reader, parameter.dataset);
  PreProcessor          preProcessor(parameter.preProcess);
  JPCCSegmentation::Ptr gmmSegmentation =
      JPCCSegmentationAdapter::build(parameter.jpccGmmSegmentation, (int)parameter.dataset.getStartFrameNumber());
  JPCCNormalEstimation normalEstimation(parameter.normalEstimation);

  JPCCEncoderAdapter encoder(parameter.jpccEncoderDynamic, parameter.jpccEncoderStatic);

  JPCCDecoderAdapter decoder;
  JPCCCombination    combination;

  JPCCContext context(parameter.jpccGmmSegmentation.resolution, parameter.jpccGmmSegmentation.type,
                      parameter.jpccGmmSegmentation.outputType, parameter.jpccEncoderDynamic.backendType,
                      parameter.jpccEncoderStatic.backendType);

  std::ofstream ofs(parameter.compressedStreamPath, std::ios::binary | std::ios::out);

  writeJPCCHeader(context.getHeader(), ofs);
  ofs.flush();
  std::ifstream ifs(parameter.compressedStreamPath, std::ios::binary | std::ios::in);
  {
    JPCCHeader header{};
    readJPCCHeader(ifs, &header);
    THROW_IF_NOT(header == context.getHeader());
    decoder.set(header);
    combination.set(header);
  }
  std::cout << __FUNCTION__ << "() "
            << "header=" << context.getHeader() << std::endl;

  {  // build gaussian mixture model
    GroupOfFrame frames;
    size_t       frameNumber    = parameter.dataset.getStartFrameNumber();
    const size_t endFrameNumber = parameter.dataset.getEndFrameNumber();
    while (!gmmSegmentation->isBuilt() && frameNumber < endFrameNumber) {
      size_t groupOfFramesSize = std::min(parameter.groupOfFramesSize, endFrameNumber - frameNumber);
      {  // load
        ScopeStopwatch clock = metric.start("Load", frameNumber);
        reader->loadAll(frameNumber, groupOfFramesSize, frames, parameter.parallel);
      }
      {  // preprocess
        ScopeStopwatch clock = metric.start("PreProcess", frameNumber);
        preProcessor.process(frames, nullptr, parameter.parallel);
      }
      GroupOfPclFrame<pcl::PointXYZINormal> pclFrames;
      {  // Convert to pcl (build)
        ScopeStopwatch clock = metric.start("ConvertToPcl (Build)", frameNumber);
        context.convertToPclBuild(parameter.parallel);
        for (const auto& frame : frames) { pclFrames.push_back(frame->toPcl<pcl::PointXYZINormal>()); }
      }
      {  // build
        ScopeStopwatch clock = metric.start("Build", frameNumber);
        gmmSegmentation->appendTrainSamplesAndBuild(frames, pclFrames, parameter.parallel);
      }

      frameNumber += groupOfFramesSize;
    }
  }

  size_t       frameNumber    = parameter.dataset.getStartFrameNumber();
  const size_t endFrameNumber = parameter.dataset.getEndFrameNumber();
  while (frameNumber < endFrameNumber) {
    size_t groupOfFramesSize = std::min(parameter.groupOfFramesSize, endFrameNumber - frameNumber);
    {  // clear
      context.clear();
    }
    {  // load
      ScopeStopwatch clock = metric.start("Load", frameNumber);
      reader->loadAll(frameNumber, groupOfFramesSize, context.getFrames(), parameter.parallel);
    }
    metric.addPoints("Raw", context.getFrames(), true);
    {  // preprocess
      ScopeStopwatch clock = metric.start("PreProcess", frameNumber);
      preProcessor.process(context.getFrames(), nullptr, parameter.parallel);
    }
    metric.addPoints("PreProcessed", context.getFrames(), true);
    {  // compute normal
      ScopeStopwatch clock = metric.start("ComputeNormal", frameNumber);
      normalEstimation.computeInPlaceAll(context.getFrames(), parameter.parallel);
    }
    {  // Convert to pcl (build)
      ScopeStopwatch clock = metric.start("ConvertToPcl (Segmentation)", frameNumber);
      context.convertToPclBuild(parameter.parallel);
    }
    {  // segmentation
      ScopeStopwatch clock = metric.start("Segmentation", frameNumber);
      gmmSegmentation->segmentation(context, parameter.parallel);
    }
    {  // convertToCoderType
      ScopeStopwatch clock = metric.start("ConvertToCoderType", frameNumber);
      encoder.convertToCoderType(context, parameter.parallel);
    }
    {  // encode
      ScopeStopwatch clock = metric.start("Encode", frameNumber);
      encoder.encode(context, parameter.parallel);
    }
    metric.addPoints("Dynamic", context.getDynamicFrames(), false);
    metric.addPoints("Static", context.getStaticFrames(), false);
    metric.addPoints("StaticAdded", context.getStaticAddedFrames(), false);
    metric.addPoints("StaticRemoved", context.getStaticRemovedFrames(), false);
    metric.addBytes("Dynamic", frameNumber, context.getDynamicEncodedBytesVector());
    metric.addBytes("Static", frameNumber, context.getStaticEncodedBytesVector());
    metric.addBytes("StaticAdded", frameNumber, context.getStaticAddedEncodedBytesVector());
    metric.addBytes("StaticRemoved", frameNumber, context.getStaticRemovedEncodedBytesVector());

    // TODO extract JPCCWriter
    {  // save
      ifs.close();
      ifs.open(parameter.compressedStreamPath, std::ios::binary | std::ios::in);
      ifs.seekg(0, std::ios::end);
      ScopeStopwatch clock = metric.start("Save", frameNumber);
      writeJPCCContext(context, ofs);
      ofs.flush();
    }
    {  // decode
      ScopeStopwatch clock = metric.start("Decode", frameNumber);
      decoder.decode(ifs, context, groupOfFramesSize);
    }
    {  // copy normal to Reconstruct
      ScopeStopwatch clock = metric.start("CopyNormalToReconstruct", frameNumber);
      metric.copyNormalToReconstruct(context, parameter.parallel);
    }
    {  // Convert to pcl (combination)
      ScopeStopwatch clock = metric.start("ConvertToPcl (Combination)", frameNumber);
      context.convertToPclCombination(parameter.parallel);
    }
    {  // combination
      ScopeStopwatch clock = metric.start("Combination", frameNumber);
      combination.combine(context, parameter.parallel);
    }
    {  // compute PSNR
      for (size_t i = 0; i < context.getPclFrames().size(); i++) {
        context.getReconstructFrames()[i]->setFrameNumber(context.getFrames()[i]->getFrameNumber());
      }
      ScopeStopwatch clock = metric.start("ComputePSNR", frameNumber);
      metric.addPSNR("A2B", context.getFrames(), context.getReconstructFrames(), parameter.parallel);
      metric.addPSNR("B2A", context.getReconstructFrames(), context.getFrames(), parameter.parallel);
    }
    if (parameter.jpccGmmSegmentation.type != jpcc::SegmentationType::NONE) {
      for (size_t i = 0; i < context.getPclFrames().size(); i++) {
        context.getDynamicReconstructFrames()[i]->setFrameNumber(context.getFrames()[i]->getFrameNumber());
      }
      // compute PSNR (Dynamic)
      ScopeStopwatch clock = metric.start("ComputePSNR (Dynamic)", frameNumber);
      metric.addPSNR("A2B (Dynamic)", context.getFrames(), context.getDynamicReconstructFrames(), parameter.parallel);
      metric.addPSNR("B2A (Dynamic)", context.getDynamicReconstructFrames(), context.getFrames(), parameter.parallel);
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
    ScopeStopwatch clock = metric.start("Wall", parameter.dataset.getStartFrameNumber());
    encode(parameter, metric);
  }

  metric.writeAndShow();

  cout << "JPCC App Encoder End" << endl;
  return 0;
}