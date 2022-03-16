#include <algorithm>
#include <array>
#include <chrono>
#include <execution>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include <vtkObject.h>

#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
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
  FramePtr<PointNormal>             staticCloud(new Frame<PointNormal>());
  FramePtr<PointNormal>             dynamicCloud(new Frame<PointNormal>());
  std::atomic_bool                  run(true);
  std::mutex                        mutex;
  std::queue<FramePtr<PointNormal>> queue;

  std::atomic_bool hasFirstFrame(false);

  auto datasetLoading = [&] {
    try {
      while (run) {
        DatasetReaderPtr<PointNormal> reader = newReader<PointNormal>(readerParameter, datasetParameter);
        GroupOfFrame<PointNormal>     frames;
        size_t                        startFrameNumber = datasetParameter.getStartFrameNumbers();
        size_t                        endFrameNumber   = startFrameNumber + datasetParameter.getFrameCounts();

        constexpr BufferIndex bufferSize  = 100;
        BufferIndex           bufferIndex = 0;
        auto                  filter = [&](const OctreeNBufBase<bufferSize, pcl::octree::OctreeContainerPointIndices,
                                               pcl::octree::OctreeContainerEmpty>::BufferPattern& bufferPattern) {
          return bufferPattern.count() > (size_t)(bufferSize * 0.1);
        };

        pcl::octree::OctreePointCloud<
            PointNormal, pcl::octree::OctreeContainerPointIndices, pcl::octree::OctreeContainerEmpty,
            OctreeNBufBase<bufferSize, pcl::octree::OctreeContainerPointIndices, pcl::octree::OctreeContainerEmpty>>
            octree(0.1);
        octree.defineBoundingBox(octree.getResolution() * 2);
        std::array<FramePtr<PointNormal>, bufferSize>   clouds;
        pcl::NormalEstimation<PointNormal, PointNormal> ne;

        pcl::search::KdTree<PointNormal>::Ptr tree(new pcl::search::KdTree<PointNormal>());
        ne.setSearchMethod(tree);
        ne.setRadiusSearch(1);

        for (BufferIndex _bufferIndex = 1; (size_t)_bufferIndex < frames.size(); _bufferIndex++) {
          clock.start();
          reader->loadAll(startFrameNumber, 1, frames, true);
          clock.stop();

          clouds.at(bufferIndex) = frames.at(0);

          ne.setInputCloud(clouds.at(bufferIndex));
          ne.compute(*clouds.at(bufferIndex));

          octree.switchBuffers(bufferIndex);
          octree.setInputCloud(clouds.at(bufferIndex));
          octree.addPointsFromInputCloud();

          startFrameNumber += 1;
          bufferIndex = (bufferIndex + 1) % bufferSize;
        }

        while (run && startFrameNumber < endFrameNumber) {
          clock.start();
          reader->loadAll(startFrameNumber, 1, frames, true);
          clock.stop();

          clouds.at(bufferIndex) = frames.at(0);

          ne.setInputCloud(clouds.at(bufferIndex));
          ne.compute(*clouds.at(bufferIndex));

          octree.switchBuffers(bufferIndex);
          octree.setInputCloud(clouds.at(bufferIndex));
          octree.addPointsFromInputCloud();

          {
            pcl::Indices          indices;
            FramePtr<PointNormal> cloud(new Frame<PointNormal>());
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
            pcl::Indices          indices;
            FramePtr<PointNormal> cloud(new Frame<PointNormal>());
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
          bufferIndex   = (bufferIndex + 1) % bufferSize;
          hasFirstFrame = true;
          std::this_thread::sleep_for(100ms);
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

  viewer->addPointCloud<PointNormal>(staticCloud, "staticCloud");
  viewer->addPointCloud<PointNormal>(dynamicCloud, "dynamicCloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "staticCloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "dynamicCloud");

  while (!viewer->wasStopped() && run) {
    viewer->spinOnce(100);
    // std::this_thread::sleep_for(100ms);
    FramePtr<PointNormal> staticCloud_;
    FramePtr<PointNormal> dynamicCloud_;
    {
      std::lock_guard<std::mutex> lock(mutex);
      staticCloud_  = staticCloud;
      dynamicCloud_ = dynamicCloud;
    }
    pcl::visualization::PointCloudColorHandlerCustom<PointNormal> staticColor(staticCloud_, 255.0, 0.0, 0.0);
    pcl::visualization::PointCloudColorHandlerCustom<PointNormal> dynamicColor(dynamicCloud_, 0.0, 0.0, 255.0);
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