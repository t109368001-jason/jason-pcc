#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include <boost/log/trivial.hpp>

#include <jpcc/common/JPCCContext.h>
#include <jpcc/combination/JPCCCombination.h>
#include <jpcc/common/ParameterParser.h>
#include <jpcc/decoder/JPCCDecoderAdapter.h>
#include <jpcc/io/Reader.h>
#include <jpcc/octree/OctreeContainerEditableIndex.h>
#include <jpcc/process/PreProcessor.h>
#include <jpcc/visualization/JPCCVisualizer.h>

#include "AppParameter.h"

using namespace std;
using namespace std::chrono;
using namespace jpcc;
using namespace jpcc::combination;
using namespace jpcc::decoder;
using namespace jpcc::io;
using namespace jpcc::octree;
using namespace jpcc::process;
using namespace jpcc::visualization;

using PointT = pcl::PointXYZ;

void main_(const AppParameter& parameter, Stopwatch& clock) {
  const auto viewer = jpcc::make_shared<JPCCVisualizer<PointT>>(parameter.visualizerParameter);

  atomic_bool run(true);
  string      primaryId = "cloud";

  viewer->addParameter(parameter);
  viewer->setPrimaryId(primaryId);

  auto loadEncoded = [&] {
    BOOST_LOG_TRIVIAL(info) << "loadEncoded" << endl;
    JPCCDecoderAdapter decoder;
    JPCCCombination    combination;

    JPCCContext context(parameter.compressedStreamPathPrefix);
    context.readHeader();
    decoder.set(context);
    combination.set(context);
    while (run && !context.anyEof()) {
      context.clear();
      {  // decode
        decoder.decode(context, parameter.groupOfFramesSize);
      }
      {  // convertFromCoderType
        decoder.convertFromCoderType(context, parameter.parallel);
      }
      {  // Convert to pcl (combination)
        context.convertToPclCombination(parameter.parallel);
      }
      {  // combination
        combination.combine(context, parameter.parallel);
      }

      viewer->enqueue(GroupOfFrameMap{{primaryId, context.getStaticFrames()}, {"dynamic", context.getDynamicFrames()}});

      while (run && viewer->isFull()) {
        this_thread::sleep_for(100ms);
      }
      context.getStartFrameNumber() += (Index)context.getFrames().size();
    }
    while (run && !viewer->isEmpty()) {
      this_thread::sleep_for(100ms);
    }
    run = false;
  };

  auto loadDataset = [&] {
    BOOST_LOG_TRIVIAL(info) << "loadDataset";
    try {
      const DatasetReader::Ptr reader = newReader(parameter.reader, parameter.dataset);
      PreProcessor             preProcessor(parameter.preProcess);

      GroupOfFrame frames;
      const auto   framesMap         = jpcc::make_shared<GroupOfFrameMap>();
      const size_t groupOfFramesSize = parameter.groupOfFramesSize;
      size_t       startFrameNumber  = parameter.dataset.getStartFrameNumber();
      const size_t endFrameNumber    = parameter.dataset.getEndFrameNumber();

      while (run && startFrameNumber < endFrameNumber) {
        clock.start();
        reader->loadAll(startFrameNumber, groupOfFramesSize, frames, parameter.parallel);
        preProcessor.process(frames, framesMap, parameter.parallel);
        framesMap->insert_or_assign(primaryId, frames);
        clock.stop();

        viewer->enqueue(*framesMap);

        while (run && viewer->isFull()) {
          this_thread::sleep_for(100ms);
        }

        if (frames.size() < groupOfFramesSize) {
          break;
        }

        startFrameNumber += groupOfFramesSize;
      }

      while (run && !viewer->isEmpty()) {
        this_thread::sleep_for(100ms);
      }

    } catch (exception& e) {
      BOOST_LOG_TRIVIAL(error) << e.what();
    }
    run = false;
  };
  shared_ptr<thread> loadThread;
  if (parameter.compressedStreamPathPrefix.empty()) {
    loadThread = make_shared<thread>(loadDataset);
  } else {
    loadThread = make_shared<thread>(loadEncoded);
  }
  while (!viewer->wasStopped() && run) {
    viewer->spinOnce(100);
    this_thread::sleep_for(100ms);
  }
  run = false;
  if (loadThread && loadThread->joinable()) {
    loadThread->join();
  }
}

int main(int argc, char* argv[]) {
  BOOST_LOG_TRIVIAL(info) << "JPCC App Dataset Viewer Start";

  AppParameter parameter;
  try {
    ParameterParser pp;
    pp.add(parameter);
    if (!pp.parse(argc, argv)) {
      return 1;
    }
    BOOST_LOG_TRIVIAL(info) << parameter;
  } catch (exception& e) {
    BOOST_LOG_TRIVIAL(error) << e.what();
    return 1;
  }

  try {
    // Timers to count elapsed wall/user time
    Stopwatch clockWall;
    Stopwatch clockUser;

    clockWall.start();
    main_(parameter, clockUser);
    clockWall.stop();

    auto totalWall = duration_cast<milliseconds>(clockWall.count()).count();
    auto totalUser = duration_cast<milliseconds>(clockUser.count()).count();
    BOOST_LOG_TRIVIAL(info) << "Processing time (wall): " << static_cast<float>(totalWall) / 1000.0 << " s";
    BOOST_LOG_TRIVIAL(info) << "Processing time (user): " << static_cast<float>(totalUser) / 1000.0 << " s";
    BOOST_LOG_TRIVIAL(info) << "Peak memory: " << getPeakMemory() << " KB";
  } catch (exception& e) {
    BOOST_LOG_TRIVIAL(error) << e.what();
  }

  BOOST_LOG_TRIVIAL(info) << "JPCC App Dataset Viewer End";
  return 0;
}