#include <array>
#include <chrono>
#include <iostream>
#include <vector>

#include <jpcc/common/Common.h>
#include <jpcc/common/ParameterParser.h>
#include <jpcc/io/Reader.h>
#include <jpcc/process/JPCCConditionalRemoval.h>
#include <jpcc/process/Process.h>
#include <jpcc/visualization/JPCCVisualizer.h>

#include "AppParameter.h"
#include "VoxelOccupancyCountToVoxelCount.h"
#include "VoxelPointCountToVoxelCount.h"
#include "VoxelPointNormalAngleSTDToVoxelCount.h"

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

void previewOnly(const AppParameter& parameter, StopwatchUserTime& clock) {
  JPCCVisualizer<PointT>::Ptr viewer = jpcc::make_shared<JPCCVisualizer<PointT>>(parameter.visualizerParameter);
  viewer->addParameter(parameter);

  const DatasetReader<PointT>::Ptr reader = newReader<PointT>(parameter.reader, parameter.dataset);

  auto backgroundFilter = jpcc::make_shared<JPCCConditionalRemoval<PointT>>(parameter.background);
  auto dynamicFilter    = jpcc::make_shared<JPCCConditionalRemoval<PointT>>(parameter.dynamic);

  GroupOfFrame<PointT> frames;
  auto                 background = jpcc::make_shared<Frame<PointT>>();
  auto                 dynamic    = jpcc::make_shared<Frame<PointT>>();
  clock.start();
  reader->loadAll(parameter.dataset.getStartFrameNumber(), 1, frames, parameter.parallel);
  clock.stop();
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

void analyze(const AppParameter& parameter, StopwatchUserTime& clock, vector<Analyzer::Ptr> analyzers) {
  analyzers.erase(std::remove_if(analyzers.begin(), analyzers.end(),
                                 [&parameter](const auto& analyzer) {
                                   if (!parameter.forceReRun && analyzer->exists()) {
                                     cout << analyzer->getFilename() << " already exists, skip analyze." << endl;
                                     return true;
                                   }
                                   return false;
                                 }),
                  analyzers.end());
  if (analyzers.empty()) { return; }

  const DatasetReader<PointT>::Ptr reader = newReader<PointT>(parameter.reader, parameter.dataset);

  auto backgroundFilter = jpcc::make_shared<JPCCConditionalRemoval<PointT>>(parameter.background);
  auto dynamicFilter    = jpcc::make_shared<JPCCConditionalRemoval<PointT>>(parameter.dynamic);

  size_t groupOfFramesSize = 32;
  size_t frameNumber       = parameter.dataset.getStartFrameNumber();
  size_t endFrameNumber    = parameter.dataset.getEndFrameNumber();

  while (frameNumber < endFrameNumber) {
    GroupOfFrame<PointT> frames;
    clock.start();
    reader->loadAll(frameNumber, groupOfFramesSize, frames, parameter.parallel);
    clock.stop();
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

      auto analyzerCompute = [&background, &dynamic, &frame](const auto& analyzer) {
        analyzer->compute(background, dynamic, frame);
      };
      if (parameter.parallel) {
        for_each(execution::par_unseq, analyzers.begin(), analyzers.end(), analyzerCompute);
      } else {
        for_each(analyzers.begin(), analyzers.end(), analyzerCompute);
      }
    }
    frameNumber += groupOfFramesSize;
  }

  auto analyzerFinalCompute = [](const auto& analyzer) { analyzer->finalCompute(); };
  if (parameter.parallel) {
    for_each(execution::par_unseq, analyzers.begin(), analyzers.end(), analyzerFinalCompute);
  } else {
    for_each(analyzers.begin(), analyzers.end(), analyzerFinalCompute);
  }
}

void main_(const AppParameter& parameter, StopwatchUserTime& clock) {
  if (parameter.previewOnly) {
    previewOnly(parameter, clock);
    return;
  }
  vector<Analyzer::Ptr> analyzers = {
      //
      jpcc::make_shared<VoxelOccupancyCountToVoxelCount>(            //
          "./bin/analyze-VoxelOccupancyCountToVoxelCount.csv",       //
          parameter.resolution),                                     //
      jpcc::make_shared<VoxelPointCountToVoxelCount>(                //
          "./bin/analyze-VoxelPointCountToVoxelCount.csv",           //
          parameter.resolution),                                     //
      jpcc::make_shared<VoxelPointNormalAngleSTDToVoxelCount>(       //
          "./bin/analyze-VoxelPointNormalAngleSTDToVoxelCount.csv",  //
          parameter.resolution),                                     //
  };
  if (parameter.analyzeParallel) {
    analyze(parameter, clock, analyzers);
  } else {
    for (const Analyzer::Ptr& analyzer : analyzers) { analyze(parameter, clock, vector<Analyzer::Ptr>{analyzer}); }
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