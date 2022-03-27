#include <chrono>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include <vtkObject.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <jpcc/common/ParameterParser.h>
#include <jpcc/io/DatasetReader.h>
#include <jpcc/process/PreProcessor.h>

#include <PCCChrono.h>
#include <PCCMemory.h>

#include "AppParameter.h"

using namespace std::chrono;
using namespace pcc;
using namespace jpcc;
using namespace jpcc::io;
using namespace jpcc::process;

void main_(const AppParameter& parameter, pcc::chrono::StopwatchUserTime& clock) {
  FramePtr<>                             cloud(new Frame<>());
  FramePtr<>                             removedCloud(new Frame<>());
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("JPCC Dataset Viewer " + parameter.dataset.name + " 0"));

  std::atomic_bool       run(true);
  std::mutex             mutex;
  std::queue<FramePtr<>> queue;
  std::queue<FramePtr<>> removedQueue;

  viewer->initCameraParameters();
  parameter.applyCameraPosition([&](double pos_x, double pos_y, double pos_z, double view_x, double view_y,
                                    double view_z, double up_x, double up_y, double up_z) {
    viewer->setCameraPosition(pos_x, pos_y, pos_z, view_x, view_y, view_z, up_x, up_y, up_z);
  });
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addCoordinateSystem(3.0, "coordinate");

  auto updateViewer = [&] {
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (queue.empty() || removedQueue.empty()) { return; }
      cloud = queue.front();
      queue.pop();
      removedCloud = removedQueue.front();
      removedQueue.pop();
    }
    viewer->setWindowName("JPCC Dataset Viewer " + parameter.dataset.name + " " + std::to_string(cloud->header.seq));
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> zColor(cloud, "z");
    if (!viewer->updatePointCloud(cloud, zColor, "cloud")) {
      viewer->addPointCloud<Point>(cloud, zColor, "cloud");
      //      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    }
    if (!viewer->updatePointCloud(removedCloud, "removedCloud")) {
      viewer->addPointCloud<Point>(removedCloud, "removedCloud");
      //      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,
      //      "removedCloud");
    }

    if (!viewer->updateText(" frame: " + std::to_string(cloud->header.seq), 5, 40, 16, 1.0, 1.0, 1.0, "frame")) {
      viewer->addText(" frame: " + std::to_string(cloud->header.seq), 5, 40, 16, 1.0, 1.0, 1.0, "frame");
    }
    if (!viewer->updateText("points: " + std::to_string(cloud->size()), 5, 20, 16, 1.0, 1.0, 1.0, "points")) {
      viewer->addText("points: " + std::to_string(cloud->size()), 5, 20, 16, 1.0, 1.0, 1.0, "points");
    }
  };
  viewer->registerKeyboardCallback([&](auto& event) {
    if (event.getKeyCode() == ' ' && event.keyDown()) { updateViewer(); }
  });
  viewer->registerPointPickingCallback([](auto& event) {
    Point point;
    event.getPoint(point.x, point.y, point.z);
    std::cout << "picked point=" << point << std::endl;
  });

  auto datasetLoading = [&] {
    try {
      DatasetReader<>::Ptr reader = newReader(parameter.reader, parameter.dataset);
      PreProcessor<>       preProcessor(parameter.preProcess);

      GroupOfFrame<>                     frames;
      PreProcessor<>::GroupOfFrameMapPtr removed(new PreProcessor<>::GroupOfFrameMap());
      size_t                             groupOfFramesSize = 32;
      size_t                             startFrameNumber  = 1;
      reader->loadAll(0, 1, frames, parameter.parallel);
      preProcessor.process(frames, removed, parameter.parallel);
      {
        std::lock_guard<std::mutex> lock(mutex);
        queue.push(frames.at(0));
        removedQueue.push(((*removed)[RADIUS_OUTLIER_REMOVAL_OPT_PREFIX])->at(0));
      }
      updateViewer();
      while (run) {
        clock.start();
        reader->loadAll(startFrameNumber, groupOfFramesSize, frames, parameter.parallel);
        preProcessor.process(frames, removed, parameter.parallel);
        clock.stop();

        do {
          {
            std::lock_guard<std::mutex> lock(mutex);
            if (queue.size() < groupOfFramesSize) {
              for (auto& frame : frames) { queue.push(frame); }
              for (auto& frame : *((*removed)[RADIUS_OUTLIER_REMOVAL_OPT_PREFIX])) { removedQueue.push(frame); }
              break;
            }
          }
          std::this_thread::sleep_for(100ms);
        } while (run);
        if (frames.size() < groupOfFramesSize) {
          startFrameNumber = 0;
          continue;
        }
        startFrameNumber += groupOfFramesSize;
      }
    } catch (std::exception& e) { std::cerr << e.what() << std::endl; }
    run = false;
  };

  shared_ptr<std::thread> thread(new std::thread(datasetLoading));
  while (!viewer->wasStopped() && run) {
    viewer->spinOnce(100);
    // std::this_thread::sleep_for(100ms);
  }
  run = false;
  if (thread && thread->joinable()) { thread->join(); }
}

int main(int argc, char* argv[]) {
  std::cout << "JPCC App Dataset Viewer Start" << std::endl;

  vtkObject::GlobalWarningDisplayOff();

  AppParameter parameter;
  try {
    ParameterParser pp;
    pp.add(parameter);
    if (!pp.parse(argc, argv)) { return 1; }
    std::cout << parameter << std::endl;
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }

  try {
    ParameterParser pp;
    // Timers to count elapsed wall/user time
    pcc::chrono::Stopwatch<steady_clock> clockWall;
    pcc::chrono::StopwatchUserTime       clockUser;

    clockWall.start();
    main_(parameter, clockUser);
    clockWall.stop();

    auto totalWall      = duration_cast<milliseconds>(clockWall.count()).count();
    auto totalUserSelf  = duration_cast<milliseconds>(clockUser.self.count()).count();
    auto totalUserChild = duration_cast<milliseconds>(clockUser.children.count()).count();
    std::cout << "Processing time (wall): " << (float)totalWall / 1000.0 << " s\n";
    std::cout << "Processing time (user.self): " << (float)totalUserSelf / 1000.0 << " s\n";
    std::cout << "Processing time (user.children): " << (float)totalUserChild / 1000.0 << " s\n";
    std::cout << "Peak memory: " << getPeakMemory() << " KB\n";
  } catch (std::exception& e) { std::cerr << e.what() << std::endl; }

  std::cout << "JPCC App Dataset Viewer End" << std::endl;
  return 0;
}