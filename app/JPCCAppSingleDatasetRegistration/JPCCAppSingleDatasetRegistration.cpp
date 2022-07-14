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
  JPCCVisualizer::Ptr viewer;
  if (!parameter.headless) {
    viewer = jpcc::make_shared<JPCCVisualizer>(parameter.visualizerParameter);
    viewer->addParameter(parameter);
  }

  const DatasetReader::Ptr reader = newReader(parameter.reader, parameter.dataset);
  PreProcessor             preProcessor(parameter.preProcess);

  pcl::Registration<PointXYZINormal, PointXYZINormal>::Ptr registration;

  if (parameter.registration == "icp") {
    auto icp     = pcl::make_shared<pcl::IterativeClosestPoint<PointXYZINormal, PointXYZINormal>>();
    registration = icp;
  } else if (parameter.registration == "ndt") {
    auto ndt = pcl::make_shared<pcl::NormalDistributionsTransform<PointXYZINormal, PointXYZINormal>>();
    ndt->setResolution(100.0);
    registration = ndt;
  }

  atomic_bool run(true);
  auto        datasetLoading = [&] {
    for (size_t datasetIndex = 0; datasetIndex < parameter.dataset.files.size(); datasetIndex++) {
      size_t                                groupOfFrames    = 32;
      size_t                                startFrameNumber = parameter.dataset.getStartFrameNumbers(datasetIndex);
      size_t                                endFrameNumber  = parameter.dataset.getEndFrameNumbers(datasetIndex);
      auto                                  cumulativeFrame = jpcc::make_shared<Frame>();
      GroupOfFrame                          frames;
      GroupOfFrameMap                       framesMap;
      JPCCOctreePointCloud<PointXYZINormal> octree(parameter.resolution);

      framesMap.insert_or_assign("cloud", GroupOfFrame{cumulativeFrame});
      octree.setInputCloud(cumulativeFrame);
      octree.addPointsFromInputCloud();

      while (startFrameNumber < endFrameNumber) {
        reader->load(datasetIndex, startFrameNumber, groupOfFrames, frames, parameter.parallel);
        for (auto& frame : frames) {
          std::cout << "reader read "
                    << "frameNumber=" << frame->header.seq << ", "
                    << "points=" << frame->size() << std::endl;
        }
        preProcessor.process(frames, nullptr, parameter.parallel);

        for (auto& frame : frames) {
          std::cout << "processing "
                    << "frameNumber=" << frame->header.seq << std::endl;
          clock.start();
          if (cumulativeFrame->size() > 0 && registration) {
            auto registeredFrame = jpcc::make_shared<Frame>();
            registration->setInputSource(cumulativeFrame);
            registration->setInputTarget(frame);
            registration->align(*registeredFrame);
            frame = registeredFrame;
          }

          for (const auto& point : frame->points) {
            if (!octree.isVoxelOccupiedAtPoint(point)) { octree.addPointToCloud(point, cumulativeFrame); }
          }
          clock.stop();

          if (viewer) {
            viewer->enqueue(framesMap);
            viewer->nextFrame();
          }
        }
        startFrameNumber += groupOfFrames;
      }
      cout << "Press any key to continue . . .";
      cin.get();
      cout << endl;
    }
    run = false;
  };

  thread datasetLoadingThread(datasetLoading);
  if (viewer) {
    while (!viewer->wasStopped()) { viewer->spinOnce(1000); }
  } else {
    while (run) { this_thread::sleep_for(milliseconds(100)); }
  }
  run = false;
  if (datasetLoadingThread.joinable()) { datasetLoadingThread.join(); }
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