#include <chrono>
#include <iostream>
#include <mutex>
#include <vector>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <jpcc/common/ParameterParser.h>
#include <jpcc/io/Reader.h>
#include <jpcc/process/PreProcessor.h>
#include <jpcc/process/OctreePointCloudOperation.h>
#include <jpcc/visualization/JPCCVisualizer.h>

#include "AppParameter.h"

using namespace std;
using namespace std::chrono;
using namespace pcc;
using namespace pcc::chrono;
using namespace jpcc;
using namespace jpcc::io;
using namespace jpcc::process;
using namespace jpcc::visualization;

using PointT = Point;

void main_(const AppParameter& parameter, StopwatchUserTime& clock) {
  JPCCVisualizer<PointT>::Ptr viewer;
  if (!parameter.headless) { viewer = jpcc::make_shared<JPCCVisualizer<PointT>>(parameter.visualizerParameter); }

  const string cloudPlaneId = "cloudPlane";
  const string cloudOtherId = "cloudOther";

  if (viewer) {
    viewer->addParameter(parameter);
    viewer->setPrimaryId(cloudOtherId);
    viewer->setColor(cloudOtherId, "z");
    viewer->setColor(cloudPlaneId, 1.0, 0.0, 1.0);
  }

  const auto framesMap = jpcc::make_shared<PreProcessor<PointT>::GroupOfFrameMap>();

  {
    const DatasetReader<PointT>::Ptr reader = newReader<PointT>(parameter.reader, parameter.dataset);
    PreProcessor<PointT>             preProcessor(parameter.preProcess);

    GroupOfFrame<PointT> frames;
    clock.start();
    reader->loadAll(parameter.dataset.getStartFrameNumber(), 1, frames, parameter.parallel);
    preProcessor.process(frames, nullptr, parameter.parallel);
    clock.stop();

    auto modelPlane = jpcc::make_shared<pcl::SampleConsensusModelPlane<Point>>(frames.at(0), true);
    auto indices    = jpcc::make_shared<Indices>();
    pcl::RandomSampleConsensus<Point> ransac(modelPlane);
    ransac.setDistanceThreshold(parameter.distanceThreshold);
    ransac.computeModel();
    ransac.getInliers(*indices);

    cout << "RANSAC iteration=" << ransac.iterations_ << endl;
    cout << "RANSAC coefficients=" << endl << ransac.model_coefficients_ << endl << endl;

    if (!viewer) { return; }

    auto framePlane = jpcc::make_shared<Frame<PointT>>();
    auto frameOther = jpcc::make_shared<Frame<PointT>>();

    process::split<Point>(frames.at(0), indices, framePlane, frameOther);

    framesMap->insert_or_assign(cloudPlaneId, GroupOfFrame<PointT>{framePlane});
    framesMap->insert_or_assign(cloudOtherId, GroupOfFrame<PointT>{frameOther});
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