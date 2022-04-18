#include <array>
#include <chrono>
#include <iostream>
#include <vector>

#define PCL_NO_PRECOMPILE
#include <pcl/octree/octree_pointcloud.h>

#include <jpcc/common/ParameterParser.h>
#include <jpcc/io/DatasetReader.h>
#include <jpcc/process/JPCCNormalEstimation.h>
#include <jpcc/process/PreProcessor.h>
#include <jpcc/process/Process.h>
#include <jpcc/octree/OctreeNBufBase.h>

#include <PCCChrono.h>
#include <PCCMemory.h>

#include "AppParameter.h"

using namespace std;
using namespace std::chrono;
using namespace pcl;
using namespace pcl::octree;
using namespace pcc;
using namespace pcc::chrono;
using namespace jpcc;
using namespace jpcc::io;
using namespace jpcc::octree;
using namespace jpcc::process;

#define BUFFER_SIZE 8

using PointT            = jpcc::PointNormal;
using LeafContainerT    = OctreeContainerPointIndices;
using BranchContainerT  = OctreeContainerEmpty;
using OctreeNBufBaseT   = OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>;
using OctreePointCloudT = OctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeNBufBaseT>;

void backgroundGenerator(const AppParameter& parameter, StopwatchUserTime& clock) {
  size_t       frameNumber    = parameter.dataset.getStartFrameNumbers();
  const size_t endFrameNumber = frameNumber + parameter.dataset.getFrameCounts();

  const DatasetReaderPtr<PointT>     reader = newReader<PointT>(parameter.reader, parameter.dataset);
  const PreProcessor<PointT>         preProcessor(parameter.preProcess);
  const JPCCNormalEstimation<PointT> normalEstimation(parameter.jpccNormalEstimation);
  OctreePointCloudT                  octree(0.1);

  octree.defineBoundingBox(octree.getResolution() * 2);

  OctreeNBufBaseT::Filter3 func = [&](const BufferIndex                     _bufferIndex,
                                      const OctreeNBufBaseT::BufferPattern& bufferPattern,
                                      const OctreeNBufBaseT::BufferIndices& bufferIndices) {
    return ((float)bufferPattern.count() > BUFFER_SIZE * 0.3);
  };

  GroupOfFrame<PointT>                 frames;
  array<FramePtr<PointT>, BUFFER_SIZE> frameBuffer;
  BufferIndex                          bufferIndex = 0;
  clock.start();
  reader->loadAll(frameNumber, BUFFER_SIZE - 1, frames, parameter.parallel);
  preProcessor.process(frames, nullptr, parameter.parallel);
  normalEstimation.computeInPlaceAll(frames, parameter.parallel);
  clock.stop();
  for (size_t i = 0; i < frames.size(); i++) {
    frameBuffer.at(bufferIndex) = frames.at(i);
    octree.switchBuffers(bufferIndex);
    octree.deleteBuffer(bufferIndex);
    octree.setInputCloud(frameBuffer.at(bufferIndex));
    octree.addPointsFromInputCloud();
  }
  frameNumber += parameter.groupOfFramesSize;

  while (frameNumber < endFrameNumber) {
    clock.start();
    reader->loadAll(frameNumber, parameter.groupOfFramesSize, frames, parameter.parallel);
    preProcessor.process(frames, nullptr, parameter.parallel);
    normalEstimation.computeInPlaceAll(frames, parameter.parallel);
    clock.stop();
    for (size_t i = 0; i < frames.size(); i++) {
      frameBuffer.at(bufferIndex) = frames.at(i);
      octree.switchBuffers(bufferIndex);
      octree.deleteBuffer(bufferIndex);
      octree.setInputCloud(frameBuffer.at(bufferIndex));
      octree.addPointsFromInputCloud();

      const auto indices       = jpcc::make_shared<Indices>();
      const auto staticCloud_  = jpcc::make_shared<Frame<PointT>>();
      const auto dynamicCloud_ = jpcc::make_shared<Frame<PointT>>();
      octree.process(func, *indices);

      process::split<PointT>(frames.at(0), indices, staticCloud_, dynamicCloud_);

      bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
    }
    frameNumber += parameter.groupOfFramesSize;
  }
}

int main(int argc, char* argv[]) {
  cout << "JPCC App Background Generator Start" << endl;

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
  Stopwatch<steady_clock> clockWall;
  StopwatchUserTime       clockUser;

  clockWall.start();
  backgroundGenerator(parameter, clockUser);
  clockWall.stop();

  auto totalWall      = duration_cast<milliseconds>(clockWall.count()).count();
  auto totalUserSelf  = duration_cast<milliseconds>(clockUser.self.count()).count();
  auto totalUserChild = duration_cast<milliseconds>(clockUser.children.count()).count();
  cout << "Processing time (wall): " << (float)totalWall / 1000.0 << " s\n";
  cout << "Processing time (user.self): " << (float)totalUserSelf / 1000.0 << " s\n";
  cout << "Processing time (user.children): " << (float)totalUserChild / 1000.0 << " s\n";
  cout << "Peak memory: " << getPeakMemory() << " KB\n";

  cout << "JPCC App Background Generator End" << endl;
  return 0;
}