#include <array>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <vector>

#include <jpcc/common/Common.h>
#include <jpcc/common/ParameterParser.h>
#include <jpcc/io/Reader.h>
#include <jpcc/process/JPCCConditionalRemoval.h>
#include <jpcc/process/JPCCNormalEstimation.h>
#include <jpcc/process/PreProcessor.h>
#include <jpcc/process/Process.h>
#include <jpcc/visualization/JPCCVisualizer.h>

#include "AppParameter.h"
#include "VoxelOccupancyCountToVoxelCount.h"
#include "VoxelPointCountToVoxelCount.h"
#include "VoxelPointNormalAngleSTDToVoxelCount.h"
#include "VoxelOccupancyIntervalSTDToVoxelCount.h"
#include "VoxelOccludedPercentageToVoxelCount.h"
#include "VoxelOccupancyChangeCountToVoxelCount.h"
#include "VoxelIntensitySTDToVoxelCount.h"
#include "VoxelPointNormalAngleEntropyToVoxelCount.h"
#include "VoxelOccupancyIntervalEntropyToVoxelCount.h"
#include "VoxelIntensityEntropyToVoxelCount.h"

using namespace std;
using namespace std::chrono;
using namespace std::filesystem;
using namespace pcl::octree;
using namespace pcc;
using namespace pcc::chrono;
using namespace jpcc;
using namespace jpcc::io;
using namespace jpcc::process;
using namespace jpcc::octree;
using namespace jpcc::visualization;

void previewOnly(const AppParameter& parameter) {
  JPCCVisualizer::Ptr viewer = jpcc::make_shared<JPCCVisualizer>(parameter.visualizerParameter);
  viewer->addParameter(parameter);

  const DatasetReader::Ptr  reader = newReader(parameter.reader, parameter.dataset);
  PreProcessor::Ptr         preProcessor;
  JPCCNormalEstimation::Ptr normalEstimation;

  if (!parameter.dataset.preProcessed) {
    preProcessor     = jpcc::make_shared<PreProcessor>(parameter.preProcess);
    normalEstimation = jpcc::make_shared<JPCCNormalEstimation>(parameter.normalEstimation);
  }

  auto backgroundFilter = jpcc::make_shared<JPCCConditionalRemoval>(parameter.background);
  auto dynamicFilter    = jpcc::make_shared<JPCCConditionalRemoval>(parameter.dynamic);

  GroupOfFrame frames;
  auto         background = jpcc::make_shared<Frame>();
  auto         dynamic    = jpcc::make_shared<Frame>();
  reader->loadAll(parameter.dataset.getStartFrameNumber(), 1, frames, parameter.parallel);
  if (!parameter.dataset.preProcessed) {
    preProcessor->process(frames, nullptr, parameter.parallel);
    normalEstimation->computeInPlaceAll(frames, parameter.parallel);
  }
  {
    auto indices = jpcc::make_shared<Indices>();
    backgroundFilter->setInputCloud(frames.at(0));
    backgroundFilter->filter(*indices);
    split(frames.at(0), indices, background, frames.at(0));
  }
  {
    auto indices = jpcc::make_shared<Indices>();
    dynamicFilter->setInputCloud(frames.at(0));
    dynamicFilter->filter(*indices);
    split(frames.at(0), indices, dynamic, frames.at(0));
  }
  viewer->enqueue({
      {"cloud", frames},                         //
      {"background", GroupOfFrame{background}},  //
      {"dynamic", GroupOfFrame{dynamic}},        //
  });
  viewer->nextFrame();
  viewer->spin();
}

void analyze(const AppParameter& parameter, StopwatchUserTime& clock, const Analyzer::Ptr& analyzer) {
  cout << analyzer->getFilepath() << " start" << endl;

  const DatasetReader::Ptr  reader = newReader(parameter.reader, parameter.dataset);
  PreProcessor::Ptr         preProcessor;
  JPCCNormalEstimation::Ptr normalEstimation;

  if (!parameter.dataset.preProcessed) {
    preProcessor     = jpcc::make_shared<PreProcessor>(parameter.preProcess);
    normalEstimation = jpcc::make_shared<JPCCNormalEstimation>(parameter.normalEstimation);
  }

  auto backgroundFilter = jpcc::make_shared<JPCCConditionalRemoval>(parameter.background);
  auto dynamicFilter    = jpcc::make_shared<JPCCConditionalRemoval>(parameter.dynamic);

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
        auto indices = jpcc::make_shared<Indices>();
        backgroundFilter->setInputCloud(frame);
        backgroundFilter->filter(*indices);
        split(frame, indices, background, frame);
      }
      {
        auto indices = jpcc::make_shared<Indices>();
        dynamicFilter->setInputCloud(frame);
        dynamicFilter->filter(*indices);
        split(frame, indices, dynamic, frame);
      }

      clock.start();
      analyzer->compute(background, dynamic, frame);
      clock.stop();
    }
    frameNumber += groupOfFramesSize;
  }

  clock.start();
  analyzer->finalCompute();
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
          if (quantResolution > resolution) { continue; }

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
        Stopwatch<steady_clock> clockWall;
        StopwatchUserTime       clockUser;

        clockWall.start();
        analyze(parameter, clockUser, analyzer);
        clockWall.stop();

        auto totalWall      = duration_cast<milliseconds>(clockWall.count()).count();
        auto totalUserSelf  = duration_cast<milliseconds>(clockUser.self.count()).count();
        auto totalUserChild = duration_cast<milliseconds>(clockUser.children.count()).count();
        cout << "Processing time (wall): " << (float)totalWall / 1000.0 << " s\n";
        cout << "Processing time (user.self): " << (float)totalUserSelf / 1000.0 << " s\n";
        cout << "Processing time (user.children): " << (float)totalUserChild / 1000.0 << " s\n";
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