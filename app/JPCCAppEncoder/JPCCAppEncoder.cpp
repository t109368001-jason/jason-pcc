#include <chrono>
#include <fstream>
#include <iostream>
#include <vector>

#include <boost/log/trivial.hpp>

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
  JPCCSegmentation::Ptr gmmSegmentation = JPCCSegmentationAdapter::build(
      parameter.jpccGmmSegmentation, static_cast<int>(parameter.dataset.getStartFrameNumber()));
  JPCCNormalEstimation normalEstimation(parameter.normalEstimation);

  JPCCEncoderAdapter encoder(parameter.jpccEncoderDynamic, parameter.jpccEncoderStatic);

  JPCCDecoderAdapter decoder;
  JPCCCombination    combination;

  JPCCContext encoderContext(parameter.compressedStreamPathPrefix);
  JPCCContext decoderContext(parameter.compressedStreamPathPrefix);

  encoderContext.writeHeader(parameter.jpccGmmSegmentation.outputType, parameter.jpccGmmSegmentation.resolution,
                             parameter.jpccEncoderDynamic.backendType, parameter.jpccEncoderStatic.backendType);
  decoderContext.readHeader();

  {
    THROW_IF_NOT(encoderContext.getSegmentationOutputType() == decoderContext.getSegmentationOutputType());
    THROW_IF_NOT(encoderContext.getDynamicContext().getHeader() == decoderContext.getDynamicContext().getHeader());
    if (encoderContext.getSegmentationOutputType() == jpcc::SegmentationOutputType::DYNAMIC_STATIC) {
      THROW_IF_NOT(encoderContext.getStaticContext().getHeader() == decoderContext.getStaticContext().getHeader());
    } else if (encoderContext.getSegmentationOutputType() ==
               jpcc::SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED) {
      THROW_IF_NOT(encoderContext.getStaticAddedContext().getHeader() ==
                   decoderContext.getStaticAddedContext().getHeader());
      THROW_IF_NOT(encoderContext.getStaticRemovedContext().getHeader() ==
                   decoderContext.getStaticRemovedContext().getHeader());
    }
    decoder.set(decoderContext);
    combination.set(decoderContext);
  }
  BOOST_LOG_TRIVIAL(info) << "dynamicHeader=" << encoderContext.getDynamicContext().getHeader();
  if (encoderContext.getSegmentationOutputType() == jpcc::SegmentationOutputType::DYNAMIC_STATIC) {
    BOOST_LOG_TRIVIAL(info) << "staticHeader=" << encoderContext.getStaticContext().getHeader();
  } else if (encoderContext.getSegmentationOutputType() ==
             jpcc::SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED) {
    BOOST_LOG_TRIVIAL(info) << "staticAddedHeader=" << encoderContext.getStaticAddedContext().getHeader();
    BOOST_LOG_TRIVIAL(info) << "staticRemovedHeader=" << encoderContext.getStaticRemovedContext().getHeader();
  }

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
      encoderContext.clear();
      decoderContext.clear();
      decoderContext.ifsSeekgEnd();
    }
    {  // load
      ScopeStopwatch clock = metric.start("Load", frameNumber);
      reader->loadAll(frameNumber, groupOfFramesSize, encoderContext.getFrames(), parameter.parallel);
    }
    metric.addPoints("Raw", encoderContext.getFrames(), true);
    {  // preprocess
      ScopeStopwatch clock = metric.start("PreProcess", frameNumber);
      preProcessor.process(encoderContext.getFrames(), nullptr, parameter.parallel);
    }
    metric.addPoints("PreProcessed", encoderContext.getFrames(), true);
    {  // compute normal
      ScopeStopwatch clock = metric.start("ComputeNormal", frameNumber);
      normalEstimation.computeInPlaceAll(encoderContext.getFrames(), parameter.parallel);
    }
    {  // Convert to pcl (build)
      ScopeStopwatch clock = metric.start("ConvertToPcl (Segmentation)", frameNumber);
      encoderContext.convertToPclBuild(parameter.parallel);
    }
    {  // segmentation
      ScopeStopwatch clock = metric.start("Segmentation", frameNumber);
      gmmSegmentation->segmentation(encoderContext, parameter.parallel);
    }
    {  // convertToCoderType
      ScopeStopwatch clock = metric.start("ConvertToCoderType", frameNumber);
      encoder.convertToCoderType(encoderContext, parameter.parallel);
    }
    {  // encode
      ScopeStopwatch clock = metric.start("Encode", frameNumber);
      encoder.encode(encoderContext, parameter.parallel);
    }
    encoderContext.flush();

    metric.addPoints("Dynamic", encoderContext.getDynamicFrames(), false);
    if (encoderContext.getSegmentationOutputType() == jpcc::SegmentationOutputType::DYNAMIC_STATIC) {
      metric.addPoints("Static", encoderContext.getStaticFrames(), false);
    } else if (encoderContext.getSegmentationOutputType() ==
               jpcc::SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED) {
      metric.addPoints("StaticAdded", encoderContext.getStaticAddedFrames(), false);
      metric.addPoints("StaticRemoved", encoderContext.getStaticRemovedFrames(), false);
    }
    metric.addBytes(
        "Dynamic", frameNumber,
        encoderContext.getDynamicContext().getOs().tellp() - decoderContext.getDynamicContext().getIs().tellg());
    if (encoderContext.getSegmentationOutputType() == jpcc::SegmentationOutputType::DYNAMIC_STATIC) {
      metric.addBytes(
          "Static", frameNumber,
          encoderContext.getStaticContext().getOs().tellp() - decoderContext.getStaticContext().getIs().tellg());
    } else if (encoderContext.getSegmentationOutputType() ==
               jpcc::SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED) {
      metric.addBytes("StaticAdded", frameNumber,
                      encoderContext.getStaticAddedContext().getOs().tellp() -
                          decoderContext.getStaticAddedContext().getIs().tellg());
      metric.addBytes("StaticRemoved", frameNumber,
                      encoderContext.getStaticRemovedContext().getOs().tellp() -
                          decoderContext.getStaticRemovedContext().getIs().tellg());
    }

    {  // decode
      ScopeStopwatch clock = metric.start("Decode", frameNumber);
      decoder.decode(decoderContext, groupOfFramesSize);
    }
    {  // convertFromCoderType
      ScopeStopwatch clock = metric.start("ConvertFromCoderType", frameNumber);
      decoder.convertFromCoderType(decoderContext, parameter.parallel);
    }
    {  // copy normal to Reconstruct
      ScopeStopwatch clock = metric.start("CopyNormalToReconstruct", frameNumber);
      metric.copyNormalToReconstruct(encoderContext, decoderContext, parameter.parallel);
    }
    {  // Convert to pcl (combination)
      ScopeStopwatch clock = metric.start("ConvertToPcl (Combination)", frameNumber);
      decoderContext.convertToPclCombination(parameter.parallel);
    }
    {  // combination
      ScopeStopwatch clock = metric.start("Combination", frameNumber);
      combination.combine(decoderContext, parameter.parallel);
    }
    {  // compute PSNR
      for (size_t i = 0; i < encoderContext.getPclFrames().size(); i++) {
        decoderContext.getFrames()[i]->setFrameNumber(encoderContext.getFrames()[i]->getFrameNumber());
      }
      ScopeStopwatch clock = metric.start("ComputePSNR", frameNumber);
      metric.addPSNR("A2B", encoderContext.getFrames(), decoderContext.getFrames(), parameter.parallel);
      metric.addPSNR("B2A", decoderContext.getFrames(), encoderContext.getFrames(), parameter.parallel);
    }
    if (parameter.jpccGmmSegmentation.type != jpcc::SegmentationType::NONE) {
      for (size_t i = 0; i < encoderContext.getPclFrames().size(); i++) {
        decoderContext.getDynamicFrames()[i]->setFrameNumber(encoderContext.getFrames()[i]->getFrameNumber());
      }
      // compute PSNR (Dynamic)
      ScopeStopwatch clock = metric.start("ComputePSNR (Dynamic)", frameNumber);
      metric.addPSNR("A2B (Dynamic)", encoderContext.getFrames(), decoderContext.getDynamicFrames(),
                     parameter.parallel);
      metric.addPSNR("B2A (Dynamic)", decoderContext.getDynamicFrames(), encoderContext.getFrames(),
                     parameter.parallel);
    }

    frameNumber += groupOfFramesSize;
  }
}

int main(int argc, char* argv[]) {
  BOOST_LOG_TRIVIAL(info) << "JPCC App Encoder Start";

  AppParameter parameter;
  try {
    ParameterParser pp;
    pp.add(parameter);
    if (!pp.parse(argc, argv)) { return 1; }
    BOOST_LOG_TRIVIAL(info) << parameter;
  } catch (exception& e) {
    BOOST_LOG_TRIVIAL(error) << e.what();
    return 1;
  }

  // Timers to count elapsed wall/user time
  JPCCMetric metric(parameter.metricParameter);

  {
    ScopeStopwatch clock = metric.start("Wall", parameter.dataset.getStartFrameNumber());
    encode(parameter, metric);
  }

  metric.writeAndShow();

  BOOST_LOG_TRIVIAL(info) << "JPCC App Encoder End";
  return 0;
}