#include <chrono>
#include <iostream>
#include <mutex>
#include <vector>

#define PCL_NO_PRECOMPILE
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <jpcc/common/ParameterParser.h>
#include <jpcc/io/DatasetReader.h>
#include <jpcc/process/PreProcessor.h>
#include <jpcc/process/OctreePointCloudOperation.h>
#include <jpcc/visualization/JPCCVisualizer.h>

#include <PCCChrono.h>
#include <PCCMemory.h>

#include "AppParameter.h"

using namespace std;
using namespace std::chrono;
using namespace pcc;
using namespace pcc::chrono;
using namespace jpcc;
using namespace jpcc::io;
using namespace jpcc::process;
using namespace jpcc::visualization;

void main_(const AppParameter& parameter, StopwatchUserTime& clock) {
  const auto viewer = jpcc::make_shared<JPCCVisualizer<>>("JPCC Ground Segmentation " + parameter.dataset.name,
                                                          parameter.visualizerParameter);

  const string cloudPlaneId = "cloudPlane";
  const string cloudOtherId = "cloudOther";

  viewer->setPrimaryId(cloudOtherId);
  viewer->setColor(cloudOtherId, "z");
  viewer->setColor(cloudPlaneId, 1.0, 0.0, 1.0);

  const auto framesMap = jpcc::make_shared<PreProcessor<>::GroupOfFrameMap>();

  {
    const DatasetReader<>::Ptr reader = newReader(parameter.reader, parameter.dataset);
    PreProcessor<>             preProcessor(parameter.preProcess);

    GroupOfFrame<> frames;
    clock.start();
    reader->loadAll(parameter.dataset.getStartFrameNumber(), 1, frames, parameter.parallel);
    preProcessor.process(frames, nullptr, parameter.parallel);
    clock.stop();

    pcl::SampleConsensusModelPlane<Point>::Ptr model_p(new pcl::SampleConsensusModelPlane<Point>(frames.at(0)));
    auto                                       indices = jpcc::make_shared<Indices>();
    pcl::RandomSampleConsensus<Point>          ransac(model_p);
    ransac.setDistanceThreshold(parameter.distanceThreshold);
    ransac.computeModel();
    ransac.getInliers(*indices);

    cout << "RANSAC coefficients=" << ransac.model_coefficients_ << endl;

    auto framePlane = jpcc::make_shared<Frame<>>();
    auto frameOther = jpcc::make_shared<Frame<>>();

    process::split<Point>(frames.at(0), indices, framePlane, frameOther);

    framesMap->insert_or_assign(cloudPlaneId, GroupOfFrame<>{framePlane});
    framesMap->insert_or_assign(cloudOtherId, GroupOfFrame<>{frameOther});
  }
  viewer->enqueue(*framesMap);
  viewer->nextFrame();

  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    // this_thread::sleep_for(100ms);
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