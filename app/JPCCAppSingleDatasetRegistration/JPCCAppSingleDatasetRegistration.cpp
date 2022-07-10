#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

#include <jpcc/common/ParameterParser.h>
#include <jpcc/io/Reader.h>
#include <jpcc/octree/JPCCOctreePointCloud.h>
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
using namespace jpcc::octree;
using namespace jpcc::process;
using namespace jpcc::visualization;

void main_(const AppParameter& parameter, StopwatchUserTime& clock) {
  const auto viewer = jpcc::make_shared<JPCCVisualizer>(parameter.visualizerParameter);

  viewer->addParameter(parameter);

  const DatasetReader::Ptr reader = newReader(parameter.reader, parameter.dataset);
  PreProcessor             preProcessor(parameter.preProcess);

  pcl::Registration<PointXYZINormal, PointXYZINormal>::Ptr registration;

  if (parameter.registration == "icp") {
    auto icp     = pcl::make_shared<pcl::IterativeClosestPoint<PointXYZINormal, PointXYZINormal>>();
    registration = icp;
  }
  if (parameter.registration == "ndt") {
    auto ndt = pcl::make_shared<pcl::NormalDistributionsTransform<PointXYZINormal, PointXYZINormal>>();
    ndt->setResolution(100.0);
    registration = ndt;
  }

  FramePtr                              cumulativeFrame;
  PreProcessor::GroupOfFrameMap         framesMap;
  JPCCOctreePointCloud<PointXYZINormal> octree(20.0);

  size_t startFrameNumber = parameter.dataset.getStartFrameNumber();
  {
    GroupOfFrame frames;
    clock.start();
    reader->load(0, startFrameNumber++, 1, frames);
    preProcessor.process(frames, nullptr, parameter.parallel);
    clock.stop();

    cumulativeFrame = frames.at(0);
    framesMap.insert_or_assign("cloud", frames);
  }
  octree.setInputCloud(cumulativeFrame);
  octree.addPointsFromInputCloud();

  viewer->registerKeyboardEvent(
      [&](const pcl::visualization::KeyboardEvent& event) {
        if (!event.keyUp()) { return false; }
        if (event.getKeyCode() == ' ') {
          cout << "processing..." << endl;

          GroupOfFrame frames;
          clock.start();
          reader->load(0, startFrameNumber++, 1, frames);
          if (frames.size() == 0) { cout << "empty" << endl; }
          std::cout << "reader read "
                    << "frameNumber=" << frames.at(0)->header.seq << ", "
                    << "points=" << frames.at(0)->size() << std::endl;
          preProcessor.process(frames, nullptr, parameter.parallel);
          clock.stop();
          if (registration) {
            auto frame = jpcc::make_shared<Frame>();
            registration->setInputSource(cumulativeFrame);
            registration->setInputTarget(frames.at(0));
            registration->align(*frame);
            frames.at(0) = frame;
          }

          for (const auto& point : frames.at(0)->points) {
            if (!octree.isVoxelOccupiedAtPoint(point)) { octree.addPointToCloud(point, cumulativeFrame); }
          }

          viewer->enqueue(framesMap);
          viewer->nextFrame();
          cout << "finished" << endl;
        }
        return true;
      },
      "");

  while (!viewer->wasStopped()) { viewer->spinOnce(100); }
}

int main(int argc, char* argv[]) {
  cout << "JPCC App Single Dataset Registration Start" << endl;

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

  cout << "JPCC App Single Dataset Registration Tool End" << endl;
  return 0;
}