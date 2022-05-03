#include <chrono>
#include <iostream>
#include <vector>
#include <map>

#define PCL_NO_PRECOMPILE
#include <pcl/octree/octree_pointcloud.h>

#include <jpcc/common/Common.h>
#include <jpcc/common/ParameterParser.h>
#include <jpcc/io/Reader.h>
#include <jpcc/process/PreProcessor.h>
#include <jpcc/octree/OctreeContainerCounter.h>

#include <PCCChrono.h>
#include <PCCMemory.h>

#include "AppParameter.h"

using namespace std;
using namespace std::chrono;
using namespace pcl::octree;
using namespace pcc;
using namespace pcc::chrono;
using namespace jpcc;
using namespace jpcc::io;
using namespace jpcc::process;
using namespace jpcc::octree;

using OctreePointCloudT = OctreePointCloud<Point,
                                           OctreeContainerCounter,
                                           OctreeContainerEmpty,
                                           OctreeBase<OctreeContainerCounter, OctreeContainerEmpty>>;

void main_(const AppParameter& parameter, StopwatchUserTime& clock) {
  OctreePointCloudT octree(parameter.resolution);

  octree.defineBoundingBox(parameter.resolution * 2);

  size_t groupOfFramesSize = 32;
  size_t frameNumber       = parameter.dataset.getStartFrameNumber();
  size_t endFrameNumber    = parameter.dataset.getEndFrameNumber();

  const DatasetReader<>::Ptr reader = newReader(parameter.reader, parameter.dataset);
  PreProcessor<>             preProcessor(parameter.preProcess);

  while (frameNumber < endFrameNumber) {
    GroupOfFrame<> frames;
    clock.start();
    reader->loadAll(frameNumber, groupOfFramesSize, frames, parameter.parallel);
    preProcessor.process(frames, nullptr, parameter.parallel);
    process::quantize(frames, parameter.resolution, parameter.parallel);
    clock.stop();
    for (auto& frame : frames) {
      octree.setInputCloud(frame);
      octree.addPointsFromInputCloud();
    }
    frameNumber += groupOfFramesSize;
  }

  map<size_t, size_t> countCounter;

  for (auto it = octree.leaf_depth_begin(), end = octree.leaf_depth_end(); it != end; ++it) {
    const size_t count = it.getLeafContainer().getCount();
    countCounter.try_emplace(count, 0);
    countCounter.at(count) = countCounter.at(count) + 1;
  }
  ofstream ofs(parameter.outputCSVPath);
  ofs << "Occupancy Count"
      << ","
      << "Count" << endl;
  for (const auto& [occupancyCount, count] : countCounter) { ofs << occupancyCount << "," << count << endl; }
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