#include <chrono>
#include <execution>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include <vtkObject.h>

#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <jpcc/common/ParameterParser.h>
#include <jpcc/io/DatasetReader.h>

#include <PCCChrono.h>
#include <PCCMemory.h>

using namespace std::chrono;
using namespace pcc;
using namespace jpcc;
using namespace jpcc::io;

void process(const DatasetParameter&         datasetParameter,
             const DatasetReaderParameter&   readerParameter,
             pcc::chrono::StopwatchUserTime& clock) {
  pcl::PointCloud<Point>::Ptr            cloud(new pcl::PointCloud<Point>());
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

  viewer->initCameraParameters();
  viewer->setCameraPosition(0.0, 0.0, 200.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 0);
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addCoordinateSystem(3.0, "coordinate");

  viewer->addPointCloud<Point>(cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

  std::atomic_bool       run(true);
  std::mutex             mutex;
  std::queue<Frame::Ptr> queue;

  auto datasetLoading = [&] {
    try {
      DatasetReader::Ptr reader = newReader(readerParameter, datasetParameter);
      GroupOfFrame       frames;
      size_t             groupOfFramesSize = 1;
      size_t             startFrameNumber  = 0;
      while (run) {
        clock.start();
        reader->loadAll(startFrameNumber, groupOfFramesSize, frames, false);
        clock.stop();
        if (frames.size() < groupOfFramesSize) {
          startFrameNumber = 0;
          continue;
        }

        pcl::PointCloud<Point>::Ptr _cloud(new pcl::PointCloud<Point>());

        std::lock_guard<std::mutex> lock(mutex);
        cloud = frames.at(0);
        std::this_thread::sleep_for(100ms);
        startFrameNumber += groupOfFramesSize;
      }
    } catch (std::exception& e) { std::cerr << e.what() << std::endl; }
    run = false;
  };

  shared_ptr<std::thread> thread(new std::thread(datasetLoading));
  while (!viewer->wasStopped() && run) {
    viewer->spinOnce(100);
    // std::this_thread::sleep_for(100ms);
    pcl::PointCloud<Point>::Ptr cloud_;
    {
      std::lock_guard<std::mutex> lock(mutex);
      cloud_ = cloud;
    }
    pcl::visualization::PointCloudColorHandlerCustom<Point> single_color(cloud_, 255.0, 0.0, 0.0);
    viewer->updatePointCloud(cloud_, single_color, "sample cloud");
  }
  run = false;
  if (thread && thread->joinable()) { thread->join(); }
}

int main(int argc, char* argv[]) {
  std::cout << "JPCC App Dataset Viewer Start" << std::endl;

  vtkObject::GlobalWarningDisplayOff();

  DatasetParameter       datasetParameter;
  DatasetReaderParameter readerParameter;
  try {
    ParameterParser pp;
    pp.add(datasetParameter);
    pp.add(readerParameter);
    if (!pp.parse(argc, argv)) { return 1; }
    std::cout << datasetParameter << std::endl;
    std::cout << readerParameter << std::endl;
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
    process(datasetParameter, readerParameter, clockUser);
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