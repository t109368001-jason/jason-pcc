#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include <jpcc/common/ParameterParser.h>
#include <jpcc/io/Reader.h>
#include <jpcc/octree/JPCCOctreePointCloud.h>
#include <jpcc/octree/OctreeContainerEditableIndex.h>
#include <jpcc/process/PreProcessor.h>
#include <jpcc/visualization/JPCCVisualizer.h>

#include "AppParameter.h"

using namespace std;
using namespace std::chrono;
using namespace pcc;
using namespace jpcc;
using namespace jpcc::io;
using namespace jpcc::octree;
using namespace jpcc::process;
using namespace jpcc::visualization;

using PointT = pcl::PointXYZ;

void main_(const AppParameter& parameter, Stopwatch& clock) {
  const auto viewer = jpcc::make_shared<JPCCVisualizer<PointT>>(parameter.visualizerParameter);

  atomic_bool run(true);
  string      primaryId = "cloud";
  string      staticId  = "static";

  viewer->addParameter(parameter);
  viewer->setPrimaryId(primaryId);

  auto datasetLoading = [&] {
    try {
      const DatasetReader<PointT>::Ptr reader = newReader<PointT>(parameter.reader, parameter.dataset);
      PreProcessor<PointT>             preProcessor(parameter.preProcess);

      FramePtr<PointT>                                                staticFrame;
      JPCCOctreePointCloud<PointT, OctreeContainerEditableIndex>::Ptr staticOctree;
      GroupOfFrame<PointT>                                            frames;
      const auto   framesMap         = jpcc::make_shared<GroupOfFrameMap<PointT>>();
      const size_t groupOfFramesSize = parameter.groupOfFramesSize;
      size_t       startFrameNumber  = parameter.dataset.getStartFrameNumber();
      const size_t endFrameNumber    = parameter.dataset.getEndFrameNumber();

      if (parameter.dataset.type == Type::PLY_SEG) {
        staticFrame = jpcc::make_shared<Frame<PointT>>();
        staticOctree =
            jpcc::make_shared<JPCCOctreePointCloud<PointT, OctreeContainerEditableIndex>>(parameter.dataset.resolution);
        staticOctree->setInputCloud(staticFrame);
      }

      while (run && startFrameNumber < endFrameNumber) {
        clock.start();
        if (parameter.dataset.type == Type::PLY_SEG) {
          GroupOfFrame<PointT> staticFrames;
          GroupOfFrame<PointT> staticAddedFrames;
          GroupOfFrame<PointT> staticRemovedFrames;
          reader->load(0, startFrameNumber, groupOfFramesSize, frames, parameter.parallel);
          reader->load(1, startFrameNumber, groupOfFramesSize, staticAddedFrames, parameter.parallel);
          reader->load(2, startFrameNumber, groupOfFramesSize, staticRemovedFrames, parameter.parallel);
#if !defined(NDEBUG)
          reader->load(3, startFrameNumber, groupOfFramesSize, staticFrames, parameter.parallel);
#endif
          for (size_t i = 0; i < staticAddedFrames.size(); i++) {
            if (staticRemovedFrames.at(i)) {
              for (const PointT& pointToRemove : staticRemovedFrames.at(i)->points) {
                staticOctree->deletePointFromCloud(pointToRemove, staticFrame);
              }
            }
            if (staticAddedFrames.at(i)) {
              for (const PointT& pointToAdd : staticAddedFrames.at(i)->points) {
                staticOctree->addPointToCloud(pointToAdd, staticFrame);
              }
            }

            assert(staticFrame->size() == staticFrames.at(i)->size());

            auto tmpFrame = jpcc::make_shared<Frame<PointT>>();
            pcl::copyPointCloud(*staticFrame, *tmpFrame);
            staticFrames.push_back(tmpFrame);
          }
          framesMap->insert_or_assign(primaryId, frames);
          framesMap->insert_or_assign(staticId, staticFrames);
        } else {
          reader->loadAll(startFrameNumber, groupOfFramesSize, frames, parameter.parallel);
          preProcessor.process(frames, framesMap, parameter.parallel);
          framesMap->insert_or_assign(primaryId, frames);
        }
        clock.stop();

        viewer->enqueue(*framesMap);

        while (run && viewer->isFull()) { this_thread::sleep_for(100ms); }

        if (frames.size() < groupOfFramesSize) { break; }

        startFrameNumber += groupOfFramesSize;
      }

      while (run && !viewer->isEmpty()) { this_thread::sleep_for(100ms); }

    } catch (exception& e) { cerr << e.what() << endl; }
    run = false;
  };

  thread datasetLoadingThread(datasetLoading);
  while (!viewer->wasStopped() && run) {
    viewer->spinOnce(100);
    this_thread::sleep_for(100ms);
  }
  run = false;
  if (datasetLoadingThread.joinable()) { datasetLoadingThread.join(); }
}

int main(int argc, char* argv[]) {
  cout << "JPCC App Dataset Viewer Start" << endl;

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
    Stopwatch clockWall;
    Stopwatch clockUser;

    clockWall.start();
    main_(parameter, clockUser);
    clockWall.stop();

    auto totalWall = duration_cast<milliseconds>(clockWall.count()).count();
    auto totalUser = duration_cast<milliseconds>(clockUser.count()).count();
    cout << "Processing time (wall): " << (float)totalWall / 1000.0 << " s\n";
    cout << "Processing time (user): " << (float)totalUser / 1000.0 << " s\n";
    cout << "Peak memory: " << getPeakMemory() << " KB\n";
  } catch (exception& e) { cerr << e.what() << endl; }

  cout << "JPCC App Dataset Viewer End" << endl;
  return 0;
}