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

#define BUFFER_SIZE 8

using namespace std::chrono;
using namespace pcc;
using namespace jpcc;
using namespace jpcc::io;
using namespace jpcc::octree;
using namespace jpcc::process;

using PointT = PointNormal;
using OctreeNBufBaseT =
    OctreeNBufBase<BUFFER_SIZE, pcl::octree::OctreeContainerPointIndices, pcl::octree::OctreeContainerEmpty>;
using OctreePointCloudT = pcl::octree::OctreePointCloud<PointT,
                                                        pcl::octree::OctreeContainerPointIndices,
                                                        pcl::octree::OctreeContainerEmpty,
                                                        OctreeNBufBaseT>;

void test(const AppParameter& parameter, pcc::chrono::StopwatchUserTime& clock) {
  FramePtr<PointT>             staticCloud(new Frame<PointT>());
  FramePtr<PointT>             dynamicCloud(new Frame<PointT>());
  std::atomic_bool             run(true);
  std::mutex                   mutex;
  std::queue<FramePtr<PointT>> staticQueue;
  std::queue<FramePtr<PointT>> dynamicQueue;

  std::atomic_bool hasFirstFrame(false);

  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

  viewer->initCameraParameters();
  parameter.applyCameraPosition([&](double pos_x, double pos_y, double pos_z, double view_x, double view_y,
                                    double view_z, double up_x, double up_y, double up_z) {
    viewer->setCameraPosition(pos_x, pos_y, pos_z, view_x, view_y, view_z, up_x, up_y, up_z);
  });
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addCoordinateSystem(3.0, "coordinate");
  viewer->addText("processing", 5, 80, 16, 1.0, 1.0, 1.0, "frame");

  auto updateViewer = [&] {
    {
      if (staticQueue.empty() || dynamicQueue.empty()) { return; }
      std::lock_guard<std::mutex> lock(mutex);
      staticCloud = staticQueue.front();
      staticQueue.pop();
      dynamicCloud = dynamicQueue.front();
      dynamicQueue.pop();
    }

    if (!viewer->updateText("frame: " + std::to_string(staticCloud->header.seq), 5, 80, 16, 1.0, 1.0, 1.0, "frame")) {
      viewer->addText("frame: " + std::to_string(staticCloud->header.seq), 5, 80, 16, 1.0, 1.0, 1.0, "frame");
    }

    pcl::visualization::PointCloudColorHandlerGenericField<PointT> staticColor(staticCloud, "z");
    if (!viewer->updatePointCloud(staticCloud, staticColor, "staticCloud")) {
      viewer->addPointCloud<PointT>(staticCloud, staticColor, "staticCloud");
    }
    if (!viewer->updateText("staticCloud points: " + std::to_string(staticCloud->size()), 5, 60, 16, 1.0, 1.0, 1.0,
                            "staticCloud points")) {
      viewer->addText("staticCloud points: " + std::to_string(staticCloud->size()), 5, 60, 16, 1.0, 1.0, 1.0,
                      "staticCloud points");
    }

    pcl::visualization::PointCloudColorHandlerCustom<PointT> dynamicColor(dynamicCloud, 255.0, 0.0, 255.0);
    if (!viewer->updatePointCloud(dynamicCloud, dynamicColor, "dynamicCloud")) {
      viewer->addPointCloud<PointT>(dynamicCloud, dynamicColor, "dynamicCloud");
    }
    if (!viewer->updateText("dynamicCloud points: " + std::to_string(dynamicCloud->size()), 5, 40, 16, 1.0, 0.0, 1.0,
                            "dynamicCloud points")) {
      viewer->addText("dynamicCloud points: " + std::to_string(dynamicCloud->size()), 5, 40, 16, 1.0, 0.0, 1.0,
                      "dynamicCloud points");
    }
  };
  viewer->registerKeyboardCallback([&](auto& event) {
    if (event.getKeyCode() == ' ' && event.keyDown()) { updateViewer(); }
  });

  auto datasetLoading = [&] {
    try {
      while (run) {
        DatasetReaderPtr<PointT> reader = newReader<PointT>(parameter.reader, parameter.dataset);
        PreProcessor<PointT>     preProcessor(parameter.preProcess);

        GroupOfFrame<PointT> frames;
        size_t               startFrameNumber = parameter.dataset.getStartFrameNumbers();
        size_t               endFrameNumber   = startFrameNumber + parameter.dataset.getFrameCounts();

        BufferIndex bufferIndex = 0;

        std::array<FramePtr<PointT>, BUFFER_SIZE> clouds;

        OctreeNBufBaseT::Filter3 func = [&](const BufferIndex                     _bufferIndex,
                                            const OctreeNBufBaseT::BufferPattern& bufferPattern,
                                            const OctreeNBufBaseT::BufferIndices& bufferIndices) {
          if ((float)bufferPattern.count() > BUFFER_SIZE * parameter.float3) { return true; }
          auto             normal = clouds.at(_bufferIndex)->at(bufferIndices.at(_bufferIndex)).getNormalVector3fMap();
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
        OctreePointCloudT octree(0.1);
        octree.defineBoundingBox(octree.getResolution() * 2);

        clock.start();
        reader->loadAll(startFrameNumber, BUFFER_SIZE - 1, frames, parameter.parallel);
        preProcessor.process(frames, nullptr, parameter.parallel);
        clock.stop();

        auto outlierRemoval = [&](const PointT& point) {
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

          pcl::NormalEstimation<PointT, PointT> ne;
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

          pcl::NormalEstimation<PointT, PointT> ne;
          ne.setRadiusSearch(0.1);
          ne.setInputCloud(clouds.at(bufferIndex));
          ne.compute(*clouds.at(bufferIndex));

          octree.switchBuffers(bufferIndex);
          octree.deleteBuffer(bufferIndex);
          octree.setInputCloud(clouds.at(bufferIndex));
          octree.addPointsFromInputCloud();

          {
            shared_ptr<pcl::Indices> indices(new pcl::Indices());
            FramePtr<PointT>         staticCloud_(new Frame<PointT>());
            FramePtr<PointT>         dynamicCloud_(new Frame<PointT>());
            octree.process(func, *indices);

            pcl::ExtractIndices<PointT> extractIndices;
            extractIndices.setInputCloud(frames.at(0));
            extractIndices.setIndices(indices);
            extractIndices.setNegative(true);
            extractIndices.filter(*dynamicCloud_);
            extractIndices.setNegative(false);
            extractIndices.filter(*staticCloud_);

            std::lock_guard<std::mutex> lock(mutex);
            staticQueue.push(staticCloud_);
            dynamicQueue.push(dynamicCloud_);
            std::cout << "staticCloud=" << *staticCloud << std::endl;
            std::cout << "dynamicCloud=" << *dynamicCloud << std::endl;
          }

          do {
            {
              std::lock_guard<std::mutex> lock(mutex);
              if (staticQueue.size() < BUFFER_SIZE) { break; }
            }
            std::this_thread::sleep_for(100ms);
          } while (run);
          if (!hasFirstFrame) {
            updateViewer();
            hasFirstFrame = true;
          }
          startFrameNumber += 1;
          bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
        }
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