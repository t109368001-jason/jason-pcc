#include <chrono>
#include <filesystem>
#include <iostream>
#include <vector>

#include <jpcc/common/Common.h>
#include <jpcc/common/ParameterParser.h>
#include <jpcc/io/Reader.h>
#include <jpcc/process/JPCCNormalEstimation.h>
#include <jpcc/process/PreProcessor.h>
#include <jpcc/process/Process.h>
#include <jpcc/visualization/JPCCVisualizer.h>

#include "AppParameter.h"
#include "analyzer/VoxelOccupancyCountToVoxelCount.h"
#include "analyzer/VoxelPointCountToVoxelCount.h"
#include "analyzer/VoxelPointNormalAngleSTDToVoxelCount.h"
#include "analyzer/VoxelOccupancyIntervalSTDToVoxelCount.h"
#include "analyzer/VoxelOccludedPercentageToVoxelCount.h"
#include "analyzer/VoxelOccupancyChangeCountToVoxelCount.h"
#include "analyzer/VoxelIntensitySTDToVoxelCount.h"
#include "analyzer/VoxelPointNormalAngleEntropyToVoxelCount.h"
#include "analyzer/VoxelOccupancyIntervalEntropyToVoxelCount.h"
#include "analyzer/VoxelIntensityEntropyToVoxelCount.h"

using namespace std;
using namespace std::chrono;
using namespace std::filesystem;
using namespace pcl::octree;
using namespace pcc;
using namespace jpcc;
using namespace jpcc::io;
using namespace jpcc::process;
using namespace jpcc::octree;
using namespace jpcc::visualization;

void previewOnly(const AppParameter& parameter) {
  typename JPCCVisualizer<PointAnalyzer>::Ptr viewer =
      jpcc::make_shared<JPCCVisualizer<PointAnalyzer>>(parameter.visualizerParameter);
  viewer->addParameter(parameter);

  const typename DatasetReader::Ptr  reader = newReader(parameter.reader, parameter.dataset);
  typename PreProcessor::Ptr         preProcessor;
  typename JPCCNormalEstimation::Ptr normalEstimation;

  if (!parameter.dataset.preProcessed) {
    preProcessor     = jpcc::make_shared<PreProcessor>(parameter.preProcess);
    normalEstimation = jpcc::make_shared<JPCCNormalEstimation>(parameter.normalEstimation);
  }

  GroupOfFrame frames;
  auto         background = jpcc::make_shared<Frame>();
  auto         dynamic    = jpcc::make_shared<Frame>();
  reader->loadAll(parameter.dataset.getStartFrameNumber(), 1, frames, parameter.parallel);
  if (!parameter.dataset.preProcessed) {
    preProcessor->process(frames, nullptr, parameter.parallel);
    normalEstimation->computeInPlaceAll(frames, parameter.parallel);
  }
  {
    Indices indices;
    conditionalRemoval(*frames[0], parameter.background.condition, indices);
    split(frames[0], indices, background, frames[0]);
  }
  {
    Indices indices;
    conditionalRemoval(*frames[0], parameter.dynamic.condition, indices);
    split(frames[0], indices, dynamic, frames[0]);
  }

  viewer->enqueue({
      {"cloud", GroupOfPclFrame<PointAnalyzer>{frames[0]->toPcl<PointAnalyzer>()}},        //
      {"background", GroupOfPclFrame<PointAnalyzer>{background->toPcl<PointAnalyzer>()}},  //
      {"dynamic", GroupOfPclFrame<PointAnalyzer>{dynamic->toPcl<PointAnalyzer>()}},        //
  });
  viewer->nextFrame();
  viewer->spin();
}

void analyze(const AppParameter& parameter, Stopwatch& clock, const Analyzer::Ptr& analyzer) {
  cout << analyzer->getFilepath() << " start" << endl;

  const typename DatasetReader::Ptr  reader = newReader(parameter.reader, parameter.dataset);
  typename PreProcessor::Ptr         preProcessor;
  typename JPCCNormalEstimation::Ptr normalEstimation;

  if (!parameter.dataset.preProcessed) {
    preProcessor     = jpcc::make_shared<PreProcessor>(parameter.preProcess);
    normalEstimation = jpcc::make_shared<JPCCNormalEstimation>(parameter.normalEstimation);
  }

  size_t groupOfFramesSize = 32;
  size_t frameNumber       = parameter.dataset.getStartFrameNumber();
  size_t endFrameNumber    = parameter.dataset.getEndFrameNumber();

  while (frameNumber < endFrameNumber) {
    GroupOfFrame frames;
    reader->loadAll(frameNumber, groupOfFramesSize, frames, parameter.parallel);
    if (!parameter.dataset.preProcessed) {
      preProcessor->process(frames, nullptr, parameter.parallel);
      normalEstimation->computeInPlaceAll(frames, parameter.parallel);
    }
    for (auto& frame : frames) {
      auto background = jpcc::make_shared<Frame>();
      auto dynamic    = jpcc::make_shared<Frame>();

      {
        Indices indices;
        conditionalRemoval(*frame, parameter.background.condition, indices);
        split(frame, indices, background, frame);
      }
      {
        Indices indices;
        conditionalRemoval(*frame, parameter.dynamic.condition, indices);
        split(frame, indices, dynamic, frame);
      }

      clock.start();
      analyzer->compute(background->toPcl<PointAnalyzer>(), dynamic->toPcl<PointAnalyzer>(),
                        frame->toPcl<PointAnalyzer>());
      clock.stop();
    }
    frameNumber += groupOfFramesSize;
  }

  clock.start();
  analyzer->finalCompute();
  analyzer->saveCloud();
  analyzer->reset();
  clock.stop();

  cout << analyzer->getFilepath() << " end" << endl;
}

void main_(AppParameter& parameter) {
  if (parameter.previewOnly) {
    previewOnly(parameter);
    return;
  }
  for (const auto& frequency : parameter.frequencies) {
    parameter.reader.frequency = frequency;
    parameter.reader.notify();
    cout << "\n========== current parameter ==========\n" << endl;
    cout << parameter << endl;
    for (const auto& resolution : parameter.resolutions) {
      vector<Analyzer::Ptr> analyzers = {
          jpcc::make_shared<VoxelOccupancyCountToVoxelCount>(frequency, resolution, parameter.outputDir),
          jpcc::make_shared<VoxelPointCountToVoxelCount>(frequency, resolution, parameter.outputDir),
          jpcc::make_shared<VoxelPointNormalAngleSTDToVoxelCount>(frequency, resolution, parameter.outputDir),
          jpcc::make_shared<VoxelOccupancyIntervalSTDToVoxelCount>(frequency, resolution, parameter.outputDir),
          jpcc::make_shared<VoxelOccupancyChangeCountToVoxelCount>(frequency, resolution, parameter.outputDir),
          jpcc::make_shared<VoxelIntensitySTDToVoxelCount>(frequency, resolution, parameter.outputDir),
          jpcc::make_shared<VoxelPointNormalAngleEntropyToVoxelCount>(frequency, resolution, parameter.outputDir),
          jpcc::make_shared<VoxelOccupancyIntervalEntropyToVoxelCount>(frequency, resolution, parameter.outputDir),
          jpcc::make_shared<VoxelIntensityEntropyToVoxelCount>(frequency, resolution, parameter.outputDir),
      };
      if (frequency == 10.0) {
        for (const auto& quantResolution : parameter.quantResolutions) {
          if ((double)quantResolution > resolution) { continue; }

          analyzers.push_back(  //
              jpcc::make_shared<VoxelOccludedPercentageToVoxelCount>(frequency, resolution, parameter.outputDir,
                                                                     quantResolution)  //
          );
        }
      }
      for (const Analyzer::Ptr& analyzer : analyzers) {
        if (!parameter.forceReRun && analyzer->exists()) {
          cout << analyzer->getFilepath() << " already exists, skip analyze." << endl;
          continue;
        }
        if (!analyzer->tryLockFile()) {
          cout << analyzer->getFilepath() << " running, skip analyze." << endl;
          continue;
        }
        Stopwatch clockWall;
        Stopwatch clockUser;

        clockWall.start();
        analyze(parameter, clockUser, analyzer);
        clockWall.stop();

        auto totalWall = duration_cast<milliseconds>(clockWall.count()).count();
        auto totalUser = duration_cast<milliseconds>(clockUser.count()).count();
        cout << "Processing time (wall): " << (float)totalWall / 1000.0 << " s\n";
        cout << "Processing time (user): " << (float)totalUser / 1000.0 << " s\n";
        cout << "Peak memory: " << getPeakMemory() << " KB\n";

        analyzer->releaseLockFile();
      }
    }
  }
}

int main(int argc, char* argv[]) {
  cout << "JPCC App Analyzer Start" << endl;

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
    ParameterParser pp;
    // Timers to count elapsed wall/user time
    main_(parameter);
  } catch (exception& e) { cerr << e.what() << endl; }

  cout << "JPCC App Analyzer End" << endl;
  return 0;
}