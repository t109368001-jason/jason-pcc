#include <chrono>
#include <iostream>
#include <vector>

#include <boost/range/counting_range.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/range/algorithm_ext.hpp>

#include <jpcc/common/JPCCContext.h>
#include <jpcc/common/ParameterParser.h>
#include <jpcc/coder/JPCCEncoderAdapter.h>
#include <jpcc/coder/JPCCDecoderAdapter.h>
#include <jpcc/io/PlyIO.h>
#include <jpcc/io/Reader.h>
#include <jpcc/metric/JPCCMetric.h>
#include <jpcc/octree/JPCCOctreePointCloud.h>
#include <jpcc/octree/OctreeContainerEditableIndex.h>
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

  JPCCEncoder<PointEncode>::Ptr dynamicEncoder = JPCCEncoderAdapter::build<PointEncode>(parameter.jpccEncoderDynamic);
  JPCCEncoder<PointEncode>::Ptr staticEncoder  = JPCCEncoderAdapter::build<PointEncode>(parameter.jpccEncoderStatic);
  JPCCEncoder<PointEncode>::Ptr staticAddedEncoder =
      JPCCEncoderAdapter::build<PointEncode>(parameter.jpccEncoderStatic);
  JPCCEncoder<PointEncode>::Ptr staticRemovedEncoder =
      JPCCEncoderAdapter::build<PointEncode>(parameter.jpccEncoderStatic);
  JPCCDecoder<PointEncode>::Ptr dynamicDecoder = JPCCDecoderAdapter::build<PointEncode>(parameter.jpccDecoderDynamic);
  JPCCDecoder<PointEncode>::Ptr staticDecoder  = JPCCDecoderAdapter::build<PointEncode>(parameter.jpccDecoderStatic);
  JPCCDecoder<PointEncode>::Ptr staticAddedDecoder =
      JPCCDecoderAdapter::build<PointEncode>(parameter.jpccDecoderStatic);
  JPCCDecoder<PointEncode>::Ptr staticRemovedDecoder =
      JPCCDecoderAdapter::build<PointEncode>(parameter.jpccDecoderStatic);

  FramePtr<PointEncode>                                                staticFrame;
  JPCCOctreePointCloud<PointEncode, OctreeContainerEditableIndex>::Ptr staticOctree;

  if (parameter.jpccGmmSegmentation.outputType == jpcc::SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED) {
    staticFrame  = jpcc::make_shared<Frame<PointEncode>>();
    staticOctree = jpcc::make_shared<JPCCOctreePointCloud<PointEncode, OctreeContainerEditableIndex>>(
        parameter.jpccGmmSegmentation.resolution);
    staticOctree->setInputCloud(staticFrame);
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

  JPCCContext<PointEncode> context;
  context.segmentationOutputType = parameter.jpccGmmSegmentation.outputType;
  while (frameNumber < endFrameNumber) {
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
    for (size_t i = 0; i < context.pclFrames.size(); i++) {
      ScopeStopwatch clock = metric.start("Encode", context.pclFrames.at(i)->header.seq);
      gmmSegmentation->segmentation(context.pclFrames.at(i), context.dynamicPclFrames.at(i),
                                    context.staticPclFrames.at(i), context.staticAddedPclFrames.at(i),
                                    context.staticRemovedPclFrames.at(i));
    }

    // encode
    for (size_t i = 0; i < context.dynamicPclFrames.size(); i++) {
      dynamicEncoder->convertFromPCL(context.dynamicPclFrames.at(i), context.dynamicFrames.at(i));
      dynamicEncoder->encode(context.dynamicFrames.at(i), context.dynamicEncodedFramesBytes.at(i));
    }
    for (size_t i = 0; i < context.staticPclFrames.size(); i++) {
      staticEncoder->convertFromPCL(context.staticPclFrames.at(i), context.staticFrames.at(i));
      staticEncoder->encode(context.staticFrames.at(i), context.staticEncodedFramesBytes.at(i));
    }
    for (size_t i = 0; i < context.staticAddedPclFrames.size(); i++) {
      staticAddedEncoder->convertFromPCL(context.staticAddedPclFrames.at(i), context.staticAddedFrames.at(i));
      staticAddedEncoder->encode(context.staticAddedFrames.at(i), context.staticAddedEncodedFramesBytes.at(i));
    }
    for (size_t i = 0; i < context.staticRemovedPclFrames.size(); i++) {
      staticRemovedEncoder->convertFromPCL(context.staticRemovedPclFrames.at(i), context.staticRemovedFrames.at(i));
      staticRemovedEncoder->encode(context.staticRemovedFrames.at(i), context.staticRemovedEncodedFramesBytes.at(i));
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
        metric.addPoints<PointEncode>("Dynamic", context.pclFrames.at(i));
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
        metric.addPoints<PointEncode>("Static", context.pclFrames.at(i));
        metric.addBytes("Static", context.pclFrames.at(i)->header.seq, context.staticEncodedFramesBytes.at(i).size());
      }
    } else {
      {  // save
        ScopeStopwatch clock = metric.start("Save", frameNumber);
        savePly<PointEncode, PointOutput>(context.staticPclFrames, parameter.outputDataset.getFilePath(0),
                                          parameter.parallel);
      }
      metric.addPoints<PointEncode>("Static", context.staticPclFrames, true);
    }
    if (parameter.jpccEncoderStatic.backendType != CoderBackendType::NONE) {
      for (size_t i = 0; i < context.staticAddedEncodedFramesBytes.size(); i++) {
        {  // save
          ScopeStopwatch clock = metric.start("Save", frameNumber);
          staticOfs.write(context.staticAddedEncodedFramesBytes.at(i).data(),
                          context.staticAddedEncodedFramesBytes.at(i).size());
        }
        metric.addPoints<PointEncode>("StaticAdded", context.pclFrames.at(i));
        metric.addBytes("StaticAdded", context.pclFrames.at(i)->header.seq,
                        context.staticAddedEncodedFramesBytes.at(i).size());
      }
    } else {
      {  // save
        ScopeStopwatch clock = metric.start("Save", frameNumber);
        savePly<PointEncode, PointOutput>(context.staticAddedPclFrames, parameter.outputDataset.getFilePath(0),
                                          parameter.parallel);
      }
      metric.addPoints<PointEncode>("StaticAdded", context.staticAddedPclFrames, true);
    }
    if (parameter.jpccEncoderStatic.backendType != CoderBackendType::NONE) {
      for (size_t i = 0; i < context.staticRemovedEncodedFramesBytes.size(); i++) {
        {  // save
          ScopeStopwatch clock = metric.start("Save", frameNumber);
          staticOfs.write(context.staticRemovedEncodedFramesBytes.at(i).data(),
                          context.staticRemovedEncodedFramesBytes.at(i).size());
        }
        metric.addPoints<PointEncode>("StaticRemoved", context.pclFrames.at(i));
        metric.addBytes("StaticRemoved", context.pclFrames.at(i)->header.seq,
                        context.staticAddedEncodedFramesBytes.at(i).size());
      }
    } else {
      {  // save
        ScopeStopwatch clock = metric.start("Save", frameNumber);
        savePly<PointEncode, PointOutput>(context.staticRemovedPclFrames, parameter.outputDataset.getFilePath(0),
                                          parameter.parallel);
      }
      metric.addPoints<PointEncode>("StaticRemoved", context.staticRemovedPclFrames, true);
    }

    // decode
    if (parameter.jpccDecoderDynamic.backendType != CoderBackendType::NONE) {
      for (size_t i = 0; i < context.dynamicEncodedFramesBytes.size(); i++) {
        dynamicDecoder->decode(context.dynamicEncodedFramesBytes.at(i), context.dynamicReconstructFrames.at(i));
        dynamicDecoder->convertToPCL(context.dynamicReconstructFrames.at(i), context.dynamicReconstructPclFrames.at(i));
      }
    } else {
      if (!parameter.parallel) {
        for (size_t i = 0; i < context.dynamicPclFrames.size(); i++) {
          context.dynamicReconstructPclFrames.at(i) = context.dynamicPclFrames.at(i);
        }
      } else {
        const auto range = boost::counting_range<size_t>(0, context.dynamicPclFrames.size());
        std::for_each(std::execution::par, range.begin(), range.end(),
                      [&](const size_t& i) {  //
                        context.dynamicReconstructPclFrames.at(i) = context.dynamicPclFrames.at(i);
                      });
      }
    }
    if (parameter.jpccDecoderStatic.backendType != CoderBackendType::NONE) {
      for (size_t i = 0; i < context.staticEncodedFramesBytes.size(); i++) {
        staticDecoder->decode(context.staticEncodedFramesBytes.at(i), context.staticReconstructFrames.at(i));
        staticDecoder->convertToPCL(context.staticReconstructFrames.at(i), context.staticReconstructPclFrames.at(i));
      }
    } else {
      if (!parameter.parallel) {
        for (size_t i = 0; i < context.staticPclFrames.size(); i++) {
          context.staticReconstructPclFrames.at(i) = context.staticPclFrames.at(i);
        }
      } else {
        const auto range = boost::counting_range<size_t>(0, context.staticPclFrames.size());
        std::for_each(std::execution::par, range.begin(), range.end(),
                      [&](const size_t& i) {  //
                        context.staticReconstructPclFrames.at(i) = context.staticPclFrames.at(i);
                      });
      }
    }
    if (parameter.jpccDecoderStatic.backendType != CoderBackendType::NONE) {
      for (size_t i = 0; i < context.staticAddedEncodedFramesBytes.size(); i++) {
        staticAddedDecoder->decode(context.staticAddedEncodedFramesBytes.at(i),
                                   context.staticAddedReconstructFrames.at(i));
        staticAddedDecoder->convertToPCL(context.staticAddedReconstructFrames.at(i),
                                         context.staticAddedReconstructPclFrames.at(i));
      }
    } else {
      if (!parameter.parallel) {
        for (size_t i = 0; i < context.staticAddedPclFrames.size(); i++) {
          context.staticAddedReconstructPclFrames.at(i) = context.staticAddedPclFrames.at(i);
        }
      } else {
        const auto range = boost::counting_range<size_t>(0, context.staticAddedPclFrames.size());
        std::for_each(std::execution::par, range.begin(), range.end(),
                      [&](const size_t& i) {  //
                        context.staticAddedReconstructPclFrames.at(i) = context.staticAddedPclFrames.at(i);
                      });
      }
    }
    if (parameter.jpccDecoderStatic.backendType != CoderBackendType::NONE) {
      for (size_t i = 0; i < context.staticRemovedEncodedFramesBytes.size(); i++) {
        staticRemovedDecoder->decode(context.staticRemovedEncodedFramesBytes.at(i),
                                     context.staticRemovedReconstructFrames.at(i));
        staticRemovedDecoder->convertToPCL(context.staticRemovedReconstructFrames.at(i),
                                           context.staticRemovedReconstructPclFrames.at(i));
      }
    } else {
      if (!parameter.parallel) {
        for (size_t i = 0; i < context.staticRemovedPclFrames.size(); i++) {
          context.staticRemovedReconstructPclFrames.at(i) = context.staticRemovedPclFrames.at(i);
        }
      } else {
        const auto range = boost::counting_range<size_t>(0, context.staticRemovedPclFrames.size());
        std::for_each(std::execution::par, range.begin(), range.end(),
                      [&](const size_t& i) {  //
                        context.staticRemovedReconstructPclFrames.at(i) = context.staticRemovedPclFrames.at(i);
                      });
      }
    }

    // reconstruct
    if (context.segmentationOutputType == jpcc::SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED) {
      context.staticReconstructPclFrames.resize(context.staticAddedReconstructPclFrames.size());
      for (size_t i = 0; i < context.staticRemovedReconstructPclFrames.size(); i++) {
        if (context.staticRemovedReconstructPclFrames.at(i)) {
          for (const PointEncode& pointToRemove : context.staticRemovedReconstructPclFrames.at(i)->points) {
            staticOctree->deletePointFromCloud(pointToRemove, staticFrame);
          }
        }
        if (context.staticAddedReconstructPclFrames.at(i)) {
          for (const PointEncode& pointToAdd : context.staticAddedReconstructPclFrames.at(i)->points) {
            staticOctree->addPointToCloud(pointToAdd, staticFrame);
          }
        }
        context.staticReconstructPclFrames.at(i) = jpcc::make_shared<Frame<PointEncode>>();
        pcl::copyPointCloud(*staticFrame, *context.staticReconstructPclFrames.at(i));
      }
    }
    for (size_t i = 0; i < context.dynamicReconstructPclFrames.size(); i++) {
      pcl::copyPointCloud(*context.dynamicReconstructPclFrames.at(i), *context.reconstructPclFrames.at(i));
    }
    for (size_t i = 0; i < context.staticReconstructPclFrames.size(); i++) {
      pcl::copyPointCloud(*context.staticReconstructPclFrames.at(i), *context.reconstructPclFrames.at(i));
    }

    GroupOfFrame<PointMetric> framesWithNormal = normalEstimation.computeAll(context.pclFrames, parameter.parallel);
    GroupOfFrame<PointMetric> reconstructFramesWithNormal =
        normalEstimation.computeAll(context.reconstructPclFrames, parameter.parallel);
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