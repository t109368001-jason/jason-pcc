#include <array>
#include <chrono>
#include <iostream>
#include <vector>
#include <map>

#include <jpcc/common/Common.h>
#include <jpcc/common/ParameterParser.h>
#include <jpcc/io/Reader.h>
#include <jpcc/process/PreProcessor.h>
#include <jpcc/process/JPCCConditionalRemoval.h>
#include <jpcc/process/JPCCNormalEstimation.h>
#include <jpcc/octree/OctreeContainerCounter.h>
#include <jpcc/octree/OctreeNBuf.h>
#include <jpcc/visualization/JPCCVisualizer.h>

#include "AppParameter.h"

#define PCL_NO_PRECOMPILE
#include <pcl/octree/octree_pointcloud.h>

using namespace std;
using namespace std::chrono;
using namespace pcl::octree;
using namespace pcc;
using namespace pcc::chrono;
using namespace jpcc;
using namespace jpcc::io;
using namespace jpcc::process;
using namespace jpcc::octree;
using namespace jpcc::visualization;

using PointT = PointNormal;

using OctreeCounterT = OctreePointCloud<PointT,
                                        OctreeContainerCounter,
                                        OctreeContainerEmpty,
                                        OctreeNBuf<3, OctreeContainerCounter, OctreeContainerEmpty>>;

void previewOnly(const AppParameter& parameter, StopwatchUserTime& clock) {
  JPCCVisualizer<PointT>::Ptr viewer = jpcc::make_shared<JPCCVisualizer<PointT>>(parameter.visualizerParameter);
  viewer->addParameter(parameter);

  const DatasetReader<PointT>::Ptr   reader = newReader<PointT>(parameter.reader, parameter.dataset);
  PreProcessor<PointT>               preProcessor(parameter.preProcess);
  const JPCCNormalEstimation<PointT> normalEstimation(parameter.jpccNormalEstimation);

  auto backgroundFilter = jpcc::make_shared<JPCCConditionalRemoval<PointT>>(parameter.background);
  auto dynamicFilter    = jpcc::make_shared<JPCCConditionalRemoval<PointT>>(parameter.dynamic);

  GroupOfFrame<PointT> frames;
  auto                 background = jpcc::make_shared<Frame<PointT>>();
  auto                 dynamic    = jpcc::make_shared<Frame<PointT>>();
  clock.start();
  reader->loadAll(parameter.dataset.getStartFrameNumber(), 1, frames, parameter.parallel);
  preProcessor.process(frames, nullptr, parameter.parallel);
  normalEstimation.computeInPlaceAll(frames, parameter.parallel);
  process::quantize(frames, parameter.resolution, parameter.parallel);
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

void main_(const AppParameter& parameter, StopwatchUserTime& clock) {
  if (parameter.previewOnly) {
    previewOnly(parameter, clock);
    return;
  }
  OctreeCounterT octreeCounter(parameter.resolution);

  octreeCounter.defineBoundingBox(parameter.resolution * 2);

  const DatasetReader<PointT>::Ptr   reader = newReader<PointT>(parameter.reader, parameter.dataset);
  PreProcessor<PointT>               preProcessor(parameter.preProcess);
  const JPCCNormalEstimation<PointT> normalEstimation(parameter.jpccNormalEstimation);

  auto backgroundFilter = jpcc::make_shared<JPCCConditionalRemoval<PointT>>(parameter.background);
  auto dynamicFilter    = jpcc::make_shared<JPCCConditionalRemoval<PointT>>(parameter.dynamic);

  size_t groupOfFramesSize = 32;
  size_t frameNumber       = parameter.dataset.getStartFrameNumber();
  size_t endFrameNumber    = parameter.dataset.getEndFrameNumber();

  while (frameNumber < endFrameNumber) {
    GroupOfFrame<PointT> frames;
    clock.start();
    reader->loadAll(frameNumber, groupOfFramesSize, frames, parameter.parallel);
    preProcessor.process(frames, nullptr, parameter.parallel);
    normalEstimation.computeInPlaceAll(frames, parameter.parallel);
    process::quantize(frames, parameter.resolution, parameter.parallel);
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

      octreeCounter.switchBuffers(0);
      octreeCounter.setInputCloud(background);
      octreeCounter.addPointsFromInputCloud();

      octreeCounter.switchBuffers(1);
      octreeCounter.setInputCloud(dynamic);
      octreeCounter.addPointsFromInputCloud();

      octreeCounter.switchBuffers(2);
      octreeCounter.setInputCloud(frame);
      octreeCounter.addPointsFromInputCloud();
    }
    frameNumber += groupOfFramesSize;
  }

  map<size_t, array<size_t, octreeCounter.getBufferSize()>> countCounter;

  for (BufferIndex bufferIndex = 0; bufferIndex < octreeCounter.getBufferSize(); bufferIndex++) {
    octreeCounter.switchBuffers(bufferIndex);
    for (auto it = octreeCounter.leaf_depth_begin(), end = octreeCounter.leaf_depth_end(); it != end; ++it) {
      const size_t count = it.getLeafContainer().getCount();
      countCounter.try_emplace(count, array<size_t, octreeCounter.getBufferSize()>{0, 0, 0});
      countCounter.at(count).at(bufferIndex) = countCounter.at(count).at(bufferIndex) + 1;
    }
  }
  ofstream ofs(parameter.outputCSVPath);
  ofs << "Occupancy Count"
      << ","
      << "Count (Background)"
      << ","
      << "Count (Dynamic)"
      << ","
      << "Count (Other)" << endl;
  for (const auto& [occupancyCount, countArray] : countCounter) {
    ofs << occupancyCount << "," << countArray.at(0) << "," << countArray.at(1) << "," << countArray.at(2) << endl;
  }
}

int main(int argc, char* argv[]) {
  cout << "JPCC App Ground Segmentation Start" << endl;

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

  cout << "JPCC App Ground Segmentation End" << endl;
  return 0;
}