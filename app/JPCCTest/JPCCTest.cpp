#include <algorithm>
#include <chrono>
#include <execution>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include <vtkObject.h>

#include <pcl/point_cloud.h>
#define PCL_NO_PRECOMPILE
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <jpcc/common/ParameterParser.h>
#include <jpcc/io/DatasetReader.h>
#include <jpcc/octree/OctreeNBufBase.h>

#include <PCCChrono.h>
#include <PCCMemory.h>

using namespace std::chrono;
using namespace pcc;
using namespace jpcc;
using namespace jpcc::io;
using namespace jpcc::octree;

void test(const DatasetParameter&         datasetParameter,
          const DatasetReaderParameter&   readerParameter,
          pcc::chrono::StopwatchUserTime& clock) {
  pcl::PointCloud<Point>::Ptr staticCloud(new pcl::PointCloud<Point>());
  pcl::PointCloud<Point>::Ptr dynamicCloud(new pcl::PointCloud<Point>());
  std::atomic_bool            run(true);
  std::mutex                  mutex;
  std::queue<Frame::Ptr>      queue;

  std::atomic_bool hasFirstFrame(false);

  auto datasetLoading = [&] {
    try {
      while (run) {
        DatasetReader::Ptr reader = newReader(readerParameter, datasetParameter);
        GroupOfFrame       frames;
        size_t             startFrameNumber = datasetParameter.getStartFrameNumbers();
        size_t             endFrameNumber   = startFrameNumber + datasetParameter.getFrameCounts();

        constexpr BufferSize bufferSize     = 1000;
        size_t               bufferSelector = 0;
        auto                 filter = [&](const OctreeNBufBase<bufferSize, pcl::octree::OctreeContainerPointIndices,
                                               pcl::octree::OctreeContainerEmpty>::BufferPattern& bufferPattern) {
          return bufferPattern.count() > (size_t)(bufferSize * 0.1);
        };

        pcl::octree::OctreePointCloud<
            pcl::PointXYZ, pcl::octree::OctreeContainerPointIndices, pcl::octree::OctreeContainerEmpty,
            OctreeNBufBase<bufferSize, pcl::octree::OctreeContainerPointIndices, pcl::octree::OctreeContainerEmpty>>
            octree(0.1);
        octree.defineBoundingBox(octree.getResolution() * 2);

        clock.start();
        reader->loadAll(startFrameNumber, bufferSize, frames, true);
        clock.stop();
        // FIXME
        for (size_t i = 1; i < frames.size(); i++) {
          octree.switchBuffers(i);
          octree.setInputCloud(frames.at(i));
          octree.addPointsFromInputCloud();
        }

        while (run && startFrameNumber < endFrameNumber) {
          clock.start();
          reader->loadAll(startFrameNumber, 1, frames, true);
          clock.stop();

          octree.switchBuffers(bufferSelector);
          octree.setInputCloud(frames.at(0));
          octree.addPointsFromInputCloud();

          {
            pcl::Indices                indices;
            pcl::PointCloud<Point>::Ptr cloud(new pcl::PointCloud<Point>());
            octree.getIndicesByFilter(filter, indices);

            cloud->clear();
            cloud->resize(indices.size());
            std::for_each(indices.begin(), indices.end(), [&, i = 0](auto index_) mutable {
              cloud->at(i) = frames.at(0)->at(index_);
              i++;
            });

            std::lock_guard<std::mutex> lock(mutex);
            staticCloud = cloud;
          }
          {
            pcl::Indices                indices;
            pcl::PointCloud<Point>::Ptr cloud(new pcl::PointCloud<Point>());
            octree.getIndicesByFilter([&](auto& bufferPattern) { return !filter(bufferPattern); }, indices);

            cloud->clear();
            cloud->resize(indices.size());
            std::for_each(indices.begin(), indices.end(), [&, i = 0](auto index_) mutable {
              cloud->at(i) = frames.at(0)->at(index_);
              i++;
            });

            std::lock_guard<std::mutex> lock(mutex);
            dynamicCloud = cloud;
          }
          {
            std::lock_guard<std::mutex> lock(mutex);
            std::cout << "staticCloud=" << *staticCloud << std::endl;
            std::cout << "dynamicCloud=" << *dynamicCloud << std::endl;
          }

          startFrameNumber += 1;
          bufferSelector = (bufferSelector + 1) % bufferSize;
          hasFirstFrame  = true;
          std::this_thread::sleep_for(100ms);
          // FIXME
          while (run) { std::this_thread::sleep_for(100ms); }
        }
      }
    } catch (std::exception& e) { std::cerr << e.what() << std::endl; }
    run = false;
  };

  shared_ptr<std::thread> thread(new std::thread(datasetLoading));

  while (!hasFirstFrame && run) { std::this_thread::sleep_for(100ms); }

  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

  viewer->initCameraParameters();
  viewer->setCameraPosition(0.0, 0.0, 200.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 0);
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addCoordinateSystem(3.0, "coordinate");

  viewer->addPointCloud<Point>(staticCloud, "staticCloud");
  viewer->addPointCloud<Point>(dynamicCloud, "dynamicCloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "staticCloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "dynamicCloud");

  while (!viewer->wasStopped() && run) {
    viewer->spinOnce(100);
    // std::this_thread::sleep_for(100ms);
    pcl::PointCloud<Point>::Ptr staticCloud_;
    pcl::PointCloud<Point>::Ptr dynamicCloud_;
    {
      std::lock_guard<std::mutex> lock(mutex);
      staticCloud_  = staticCloud;
      dynamicCloud_ = dynamicCloud;
    }
    pcl::visualization::PointCloudColorHandlerCustom<Point> staticColor(staticCloud_, 255.0, 0.0, 0.0);
    pcl::visualization::PointCloudColorHandlerCustom<Point> dynamicColor(dynamicCloud_, 0.0, 0.0, 255.0);
    viewer->updatePointCloud(staticCloud_, staticColor, "staticCloud");
    viewer->updatePointCloud(dynamicCloud_, dynamicColor, "dynamicCloud");
  }
  run = false;
  if (thread && thread->joinable()) { thread->join(); }
}

int main(int argc, char* argv[]) {
  std::cout << "JPCC Test App Start" << std::endl;

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

  ParameterParser pp;
  // Timers to count elapsed wall/user time
  pcc::chrono::Stopwatch<steady_clock> clockWall;
  pcc::chrono::StopwatchUserTime       clockUser;

  clockWall.start();
  test(datasetParameter, readerParameter, clockUser);
  clockWall.stop();

  auto totalWall      = duration_cast<milliseconds>(clockWall.count()).count();
  auto totalUserSelf  = duration_cast<milliseconds>(clockUser.self.count()).count();
  auto totalUserChild = duration_cast<milliseconds>(clockUser.children.count()).count();
  std::cout << "Processing time (wall): " << (float)totalWall / 1000.0 << " s\n";
  std::cout << "Processing time (user.self): " << (float)totalUserSelf / 1000.0 << " s\n";
  std::cout << "Processing time (user.children): " << (float)totalUserChild / 1000.0 << " s\n";
  std::cout << "Peak memory: " << getPeakMemory() << " KB\n";

  std::cout << "JPCC Test App End" << std::endl;
  return 0;
}