#include <algorithm>
#include <array>
#include <chrono>
#include <execution>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

#include <vtkObject.h>

#include <Eigen/Dense>

#include <boost/range/counting_range.hpp>

#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/octree/octree_pointcloud.h>

#include <jpcc/common/ParameterParser.h>
#include <jpcc/io/DatasetReader.h>
#include <jpcc/octree/OctreeNBufBase.h>
#include <jpcc/process/PreProcessor.h>
#include <jpcc/visualization/JPCCVisualizer.h>

#include <PCCChrono.h>
#include <PCCMemory.h>

#include "AppParameter.h"

#define BUFFER_SIZE 8

using namespace std;
using namespace std::chrono;
using namespace Eigen;
using namespace pcl;
using namespace pcl::octree;
using namespace pcc;
using namespace pcc::chrono;
using namespace jpcc;
using namespace jpcc::io;
using namespace jpcc::octree;
using namespace jpcc::process;
using namespace jpcc::visualization;

using PointT            = jpcc::PointNormal;
using OctreeNBufBaseT   = OctreeNBufBase<BUFFER_SIZE, OctreeContainerPointIndices, OctreeContainerEmpty>;
using OctreePointCloudT = OctreePointCloud<PointT, OctreeContainerPointIndices, OctreeContainerEmpty, OctreeNBufBaseT>;

void test(const AppParameter& parameter, StopwatchUserTime& clock) {
  const auto viewer = jpcc::make_shared<JPCCVisualizer<PointT>>("JPCC Dataset Viewer " + parameter.dataset.name,
                                                                parameter.visualizerParameter);

  atomic_bool  run(true);
  atomic_bool  hasFirstFrame(false);
  const string primaryId = "static";
  const string dynamicId = "dynamic";
  viewer->setPrimaryId(primaryId);
  viewer->setColor(primaryId, "z");
  viewer->setColor(dynamicId, 1.0, 1.0, 1.0);
  viewer->setColor(RADIUS_OUTLIER_REMOVAL_OPT_PREFIX, 0.5, 0.0, 1.0);
  viewer->setColor(STATISTICAL_OUTLIER_REMOVAL_OPT_PREFIX, 0.5, 0.0, 0.5);
  viewer->setColor(JPCC_CONDITIONAL_REMOVAL_OPT_PREFIX, 0.5, 0.5, 0.5);

  auto datasetLoading = [&] {
    try {
      while (run) {
        const DatasetReaderPtr<PointT> reader = newReader<PointT>(parameter.reader, parameter.dataset);
        PreProcessor<PointT>           preProcessor(parameter.preProcess);

        GroupOfFrame<PointT> frames;
        const auto           framesMap = jpcc::make_shared<JPCCVisualizer<PointT>::GroupOfFrameMap>();

        size_t       startFrameNumber = parameter.dataset.getStartFrameNumbers();
        const size_t endFrameNumber   = startFrameNumber + parameter.dataset.getFrameCounts();

        BufferIndex bufferIndex = 0;

        array<FramePtr<PointT>, BUFFER_SIZE> frameBuffer;

        OctreeNBufBaseT::Filter3 func = [&](const BufferIndex                     _bufferIndex,
                                            const OctreeNBufBaseT::BufferPattern& bufferPattern,
                                            const OctreeNBufBaseT::BufferIndices& bufferIndices) {
          if ((float)bufferPattern.count() > BUFFER_SIZE * parameter.float3) { return true; }
          auto      normal = frameBuffer.at(_bufferIndex)->at(bufferIndices.at(_bufferIndex)).getNormalVector3fMap();
          Matrix3Xf matrix(3, bufferPattern.count());
          int       i = 0;
          for (BufferIndex ii = 0; ii < BUFFER_SIZE; ii++) {
            if (bufferPattern.test(ii)) {
              matrix.col(i++) = frameBuffer.at(ii)->at(bufferIndices.at(ii)).getNormalVector3fMap();
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

        auto range      = boost::counting_range<size_t>(0, frames.size());
        auto calcNormal = [&](const FramePtr<PointT>& frame) {
          NormalEstimation<PointT, PointT> ne;
          //          ne.setRadiusSearch(parameter.float1);
          ne.setKSearch(parameter.int1);
          ne.setInputCloud(frame);
          ne.compute(*frame);
        };
        std::copy(frames.begin(), frames.end(), frameBuffer.begin());
        if (parameter.parallel) {
          for_each(execution::par_unseq, range.begin(), range.end(),
                   [&](const size_t i) { calcNormal(frameBuffer.at(i)); });
        } else {
          for_each(range.begin(), range.end(), [&](const size_t i) { calcNormal(frameBuffer.at(i)); });
        }

        for (size_t i = 0; i < frames.size(); i++) {
          try {
            octree.switchBuffers(bufferIndex);
            octree.deleteBuffer(bufferIndex);
            octree.setInputCloud(frameBuffer.at(bufferIndex));
            octree.addPointsFromInputCloud();

            startFrameNumber += 1;
            bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
          } catch (exception& e) { cerr << e.what() << endl; }
        }

        while (run && startFrameNumber < endFrameNumber) {
          clock.start();
          reader->loadAll(startFrameNumber, 1, frames, parameter.parallel);
          preProcessor.process(frames, framesMap, parameter.parallel);
          clock.stop();

          frameBuffer.at(bufferIndex) = frames.at(0);
          calcNormal(frameBuffer.at(bufferIndex));

          octree.switchBuffers(bufferIndex);
          octree.deleteBuffer(bufferIndex);
          octree.setInputCloud(frameBuffer.at(bufferIndex));
          octree.addPointsFromInputCloud();

          {
            const auto indices       = jpcc::make_shared<Indices>();
            const auto staticCloud_  = jpcc::make_shared<Frame<PointT>>();
            const auto dynamicCloud_ = jpcc::make_shared<Frame<PointT>>();
            octree.process(func, *indices);

            process::split<PointT>(frames.at(0), indices, staticCloud_, dynamicCloud_);

            framesMap->insert_or_assign(primaryId, GroupOfFrame<PointT>{staticCloud_});
            framesMap->insert_or_assign(dynamicId, GroupOfFrame<PointT>{dynamicCloud_});

            viewer->enqueue(*framesMap);
          }

          while (run && viewer->isFull()) { this_thread::sleep_for(100ms); }

          if (!hasFirstFrame) {
            viewer->nextFrame();
            hasFirstFrame = true;
          }
          startFrameNumber += 1;
          bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
        }
      }
    } catch (exception& e) { cerr << e.what() << endl; }
    run = false;
  };

  auto datasetLoadingThread(jpcc::make_shared<thread>(datasetLoading));

  while (!viewer->wasStopped() && run) {
    viewer->spinOnce(100);
    // this_thread::sleep_for(100ms);
  }
  run = false;
  if (datasetLoadingThread && datasetLoadingThread->joinable()) { datasetLoadingThread->join(); }
}

int main(int argc, char* argv[]) {
  cout << "JPCC Test App Start" << endl;

  vtkObject::GlobalWarningDisplayOff();

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

  ParameterParser pp;
  // Timers to count elapsed wall/user time
  Stopwatch<steady_clock> clockWall;
  StopwatchUserTime       clockUser;

  clockWall.start();
  test(parameter, clockUser);
  clockWall.stop();

  auto totalWall      = duration_cast<milliseconds>(clockWall.count()).count();
  auto totalUserSelf  = duration_cast<milliseconds>(clockUser.self.count()).count();
  auto totalUserChild = duration_cast<milliseconds>(clockUser.children.count()).count();
  cout << "Processing time (wall): " << (float)totalWall / 1000.0 << " s\n";
  cout << "Processing time (user.self): " << (float)totalUserSelf / 1000.0 << " s\n";
  cout << "Processing time (user.children): " << (float)totalUserChild / 1000.0 << " s\n";
  cout << "Peak memory: " << getPeakMemory() << " KB\n";

  cout << "JPCC Test App End" << endl;
  return 0;
}