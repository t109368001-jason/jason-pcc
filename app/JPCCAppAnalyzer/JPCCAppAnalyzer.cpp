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

using PointT = PointNormal;

void previewOnly(const AppParameter& parameter) {
  JPCCVisualizer<PointT>::Ptr viewer = jpcc::make_shared<JPCCVisualizer<PointT>>(parameter.visualizerParameter);
  viewer->addParameter(parameter);

  const DatasetReader<PointT>::Ptr  reader = newReader<PointT>(parameter.reader, parameter.dataset);
  PreProcessor<PointT>::Ptr         preProcessor;
  JPCCNormalEstimation<PointT>::Ptr normalEstimation;

  if (!parameter.dataset.preProcessed) {
    preProcessor     = jpcc::make_shared<PreProcessor<PointT>>(parameter.preProcess);
    normalEstimation = jpcc::make_shared<JPCCNormalEstimation<PointT>>(parameter.normalEstimation);
  }

  auto backgroundFilter = jpcc::make_shared<JPCCConditionalRemoval<PointT>>(parameter.background);
  auto dynamicFilter    = jpcc::make_shared<JPCCConditionalRemoval<PointT>>(parameter.dynamic);

  GroupOfFrame<PointT> frames;
  auto                 background = jpcc::make_shared<Frame<PointT>>();
  auto                 dynamic    = jpcc::make_shared<Frame<PointT>>();
  reader->loadAll(parameter.dataset.getStartFrameNumber(), 1, frames, parameter.parallel);
  if (!parameter.dataset.preProcessed) {
    preProcessor->process(frames, nullptr, parameter.parallel);
    normalEstimation->computeInPlaceAll(frames, parameter.parallel);
  }
  {
    auto indices = jpcc::make_shared<Indices>();
    backgroundFilter->setInputCloud(frames.at(0));
    backgroundFilter->filter(*indices);
    split<PointT>(frames.at(0), indices, background, frames.at(0));
  }
  {
    auto indices = jpcc::make_shared<Indices>();
    dynamicFilter->setInputCloud(frames.at(0));
    dynamicFilter->filter(*indices);
    split<PointT>(frames.at(0), indices, dynamic, frames.at(0));
  }
  viewer->enqueue({
      {"cloud", frames},                                 //
      {"background", GroupOfFrame<PointT>{background}},  //
      {"dynamic", GroupOfFrame<PointT>{dynamic}},        //
  });
  viewer->nextFrame();
  viewer->spin();
}

void analyze(const AppParameter& parameter, StopwatchUserTime& clock, const Analyzer::Ptr& analyzer) {
  if (!parameter.forceReRun && analyzer->exists()) {
    cout << analyzer->getFilepath() << " already exists, skip analyze." << endl;
    return;
  }
  if (!analyzer->tryLockFile()) {
    cout << analyzer->getFilepath() << " running, skip analyze." << endl;
    return;
  }

  const DatasetReader<PointT>::Ptr  reader = newReader<PointT>(parameter.reader, parameter.dataset);
  PreProcessor<PointT>::Ptr         preProcessor;
  JPCCNormalEstimation<PointT>::Ptr normalEstimation;

  if (!parameter.dataset.preProcessed) {
    preProcessor     = jpcc::make_shared<PreProcessor<PointT>>(parameter.preProcess);
    normalEstimation = jpcc::make_shared<JPCCNormalEstimation<PointT>>(parameter.normalEstimation);
  }

  auto backgroundFilter = jpcc::make_shared<JPCCConditionalRemoval<PointT>>(parameter.background);
  auto dynamicFilter    = jpcc::make_shared<JPCCConditionalRemoval<PointT>>(parameter.dynamic);

  size_t groupOfFramesSize = 32;
  size_t frameNumber       = parameter.dataset.getStartFrameNumber();
  size_t endFrameNumber    = parameter.dataset.getEndFrameNumber();

  while (frameNumber < endFrameNumber) {
    GroupOfFrame<PointT> frames;
    reader->loadAll(frameNumber, groupOfFramesSize, frames, parameter.parallel);
    if (!parameter.dataset.preProcessed) {
      preProcessor->process(frames, nullptr, parameter.parallel);
      normalEstimation->computeInPlaceAll(frames, parameter.parallel);
    }
    for (auto& frame : frames) {
      auto background = jpcc::make_shared<Frame<PointT>>();
      auto dynamic    = jpcc::make_shared<Frame<PointT>>();
      {
        auto indices = jpcc::make_shared<Indices>();
        backgroundFilter->setInputCloud(frame);
        backgroundFilter->filter(*indices);
        split<PointT>(frame, indices, background, frame);
      }
      {
        auto indices = jpcc::make_shared<Indices>();
        dynamicFilter->setInputCloud(frame);
        dynamicFilter->filter(*indices);
        split<PointT>(frame, indices, dynamic, frame);
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
}

void main_(AppParameter& parameter, StopwatchUserTime& clock) {
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
      for (const Analyzer::Ptr& analyzer : analyzers) { analyze(parameter, clock, analyzer); }
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
    Stopwatch<steady_clock> clockWall;
    StopwatchUserTime       clockUser;

    clockWall.start();
    main_(parameter, clockUser);
    clockWall.stop();

    auto totalWall      = duration_cast<milliseconds>(clockWall.count()).count();
    auto totalUserSelf  = duration_cast<milliseconds>(clockUser.self.count()).count();
    auto totalUserChild = duration_cast<milliseconds>(clockUser.children.count()).count();
    cout << "Processing time (wall): " << (float)totalWall / 1000.0 << " s\n";
    cout << "Processing time (user.self): " << (float)totalUserSelf / 1000.0 << " s\n";
    cout << "Processing time (user.children): " << (float)totalUserChild / 1000.0 << " s\n";
    cout << "Peak memory: " << getPeakMemory() << " KB\n";
  } catch (exception& e) { cerr << e.what() << endl; }

  cout << "JPCC App Analyzer End" << endl;
  return 0;
}