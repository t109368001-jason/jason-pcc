#include <algorithm>
#include <array>
#include <chrono>
#include <execution>
#include <future>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include <vtkObject.h>

#include <Eigen/Dense>

#include <boost/range/counting_range.hpp>

#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <jpcc/common/ParameterParser.h>
#include <jpcc/io/DatasetReader.h>
#include <jpcc/octree/OctreeNBufBase.h>
#include <jpcc/process/PreProcessor.h>

#include <PCCChrono.h>
#include <PCCMemory.h>

#include "AppParameter.h"

#define BUFFER_SIZE 10

using namespace std::chrono;
using namespace pcc;
using namespace jpcc;
using namespace jpcc::io;
using namespace jpcc::octree;
using namespace jpcc::process;

void test(const AppParameter& parameter, pcc::chrono::StopwatchUserTime& clock) {
  FramePtr<PointNormal>             staticCloud(new Frame<PointNormal>());
  FramePtr<PointNormal>             dynamicCloud(new Frame<PointNormal>());
  std::atomic_bool                  run(true);
  std::mutex                        mutex;
  std::queue<FramePtr<PointNormal>> queue;

  std::atomic_bool hasFirstFrame(false);

  auto datasetLoading = [&] {
    try {
      while (run) {
        DatasetReaderPtr<PointNormal> reader = newReader<PointNormal>(parameter.reader, parameter.dataset);
        PreProcessor<PointNormal>     preProcessor(parameter.preProcess);

        GroupOfFrame<PointNormal> frames;
        size_t                    startFrameNumber = parameter.dataset.getStartFrameNumbers();
        size_t                    endFrameNumber   = startFrameNumber + parameter.dataset.getFrameCounts();

        BufferIndex bufferIndex = 0;

        std::array<FramePtr<PointNormal>, BUFFER_SIZE> clouds;

        std::function<bool(const BufferIndex                                                       bufferIndex,
                           const OctreeNBufBase<BUFFER_SIZE, pcl::octree::OctreeContainerPointIndices,
                                                pcl::octree::OctreeContainerEmpty>::BufferPattern& bufferPattern,
                           const std::array<int, BUFFER_SIZE>&                                     bufferIndices)>
            func = [&](const BufferIndex                                                       _bufferIndex,
                       const OctreeNBufBase<BUFFER_SIZE, pcl::octree::OctreeContainerPointIndices,
                                            pcl::octree::OctreeContainerEmpty>::BufferPattern& bufferPattern,
                       const std::array<int, BUFFER_SIZE>&                                     bufferIndices) {
              if ((float)bufferPattern.count() > BUFFER_SIZE * parameter.float3) { return true; }
              auto normal = clouds.at(_bufferIndex)->at(bufferIndices.at(_bufferIndex)).getNormalVector3fMap();
              Eigen::Matrix3Xf matrix(3, bufferPattern.count());
              int              i = 0;
              for (BufferIndex ii = 0; ii < BUFFER_SIZE; ii++) {
                if (bufferPattern.test(ii)) {
                  matrix.col(i++) = clouds.at(ii)->at(bufferIndices.at(ii)).getNormalVector3fMap();
                }
              }
              float e = (float)((matrix.transpose() * normal).array().acos().mean() / M_PI * 180.0);
              return e < parameter.float2;
            };
        std::function<bool(const BufferIndex                                                       bufferIndex,
                           const OctreeNBufBase<BUFFER_SIZE, pcl::octree::OctreeContainerPointIndices,
                                                pcl::octree::OctreeContainerEmpty>::BufferPattern& bufferPattern,
                           const std::array<int, BUFFER_SIZE>&                                     bufferIndices)>
            notFunc = [&](const BufferIndex                                                       _bufferIndex,
                          const OctreeNBufBase<BUFFER_SIZE, pcl::octree::OctreeContainerPointIndices,
                                               pcl::octree::OctreeContainerEmpty>::BufferPattern& bufferPattern,
                          const std::array<int, BUFFER_SIZE>&                                     bufferIndices) {
              bool result = func(_bufferIndex, bufferPattern, bufferIndices);
              return !result;
            };

        pcl::octree::OctreePointCloud<
            PointNormal, pcl::octree::OctreeContainerPointIndices, pcl::octree::OctreeContainerEmpty,
            OctreeNBufBase<BUFFER_SIZE, pcl::octree::OctreeContainerPointIndices, pcl::octree::OctreeContainerEmpty>>
            octree(0.1);
        octree.defineBoundingBox(octree.getResolution() * 2);

        clock.start();
        reader->loadAll(startFrameNumber, BUFFER_SIZE - 1, frames, parameter.parallel);
        preProcessor.process(frames, nullptr, parameter.parallel);
        clock.stop();

        auto outlierRemoval = [&](const PointNormal& point) {
          float distance = point.getVector3fMap().norm();
          return distance > 100.0;
        };
        for (auto& frame : frames) {
          if (parameter.parallel) {
            frame->erase(std::remove_if(std::execution::par_unseq, frame->begin(), frame->end(), outlierRemoval));
          } else {
            frame->erase(std::remove_if(frame->begin(), frame->end(), outlierRemoval));
          }
        }

        auto range      = boost::counting_range<size_t>(0, frames.size());
        auto calcNormal = [&](size_t i) {
          clouds.at(i) = frames.at(i);

          pcl::NormalEstimation<PointNormal, PointNormal> ne;
          ne.setRadiusSearch(parameter.float1);
          ne.setInputCloud(clouds.at(i));
          ne.compute(*clouds.at(i));
        };

        if (parameter.parallel) {
          std::for_each(std::execution::par_unseq, range.begin(), range.end(), calcNormal);
        } else {
          std::for_each(range.begin(), range.end(), calcNormal);
        }

        for (size_t i = 0; i < frames.size(); i++) {
          try {
            octree.switchBuffers(bufferIndex);
            octree.deleteBuffer(bufferIndex);
            octree.setInputCloud(clouds.at(bufferIndex));
            octree.addPointsFromInputCloud();

            startFrameNumber += 1;
            bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
          } catch (std::exception& e) { std::cerr << e.what() << std::endl; }
        }

        while (run && startFrameNumber < endFrameNumber) {
          clock.start();
          reader->loadAll(startFrameNumber, 1, frames, parameter.parallel);
          preProcessor.process(frames, nullptr, parameter.parallel);
          clock.stop();

          clouds.at(bufferIndex) = frames.at(0);

          pcl::NormalEstimation<PointNormal, PointNormal> ne;
          ne.setRadiusSearch(0.1);
          ne.setInputCloud(clouds.at(bufferIndex));
          ne.compute(*clouds.at(bufferIndex));

          octree.switchBuffers(bufferIndex);
          octree.deleteBuffer(bufferIndex);
          octree.setInputCloud(clouds.at(bufferIndex));
          octree.addPointsFromInputCloud();

          {
            pcl::Indices          indices;
            FramePtr<PointNormal> cloud(new Frame<PointNormal>());
            octree.process(func, indices);

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
            octree.process(notFunc, indices);

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
          bufferIndex   = (bufferIndex + 1) % BUFFER_SIZE;
          hasFirstFrame = true;
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
  //  viewer->setCameraPosition(0.0, 0.0, 200.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0);
  viewer->setCameraPosition(-10, 0.0, 0.0,  //
                            1.0, 0.0, 0.0,  //
                            0.0, 0.0, 1.0);
  viewer->setBackgroundColor(0, 0, 0);
  //  viewer->addCoordinateSystem(3.0, "coordinate");

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

  ParameterParser pp;
  // Timers to count elapsed wall/user time
  pcc::chrono::Stopwatch<steady_clock> clockWall;
  pcc::chrono::StopwatchUserTime       clockUser;

  clockWall.start();
  test(parameter, clockUser);
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