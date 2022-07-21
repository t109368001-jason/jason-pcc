#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

#include <pcl/io/ply_io.h>

#include <jpcc/common/ParameterParser.h>
#include <jpcc/io/Reader.h>
#include <jpcc/process/PreProcessor.h>
#include <jpcc/visualization/JPCCVisualizer.h>

#include "AppParameter.h"

using namespace std;
using namespace std::chrono;
using namespace pcc;
using namespace pcc::chrono;
using namespace jpcc;
using namespace jpcc::io;
using namespace jpcc::process;
using namespace jpcc::visualization;

void main_(const AppParameter& parameter, StopwatchUserTime& clock) {
  const auto viewer = jpcc::make_shared<JPCCVisualizer>(parameter.visualizerParameter);

  atomic_bool run(true);
  string      primaryId = "cloud";
  string      staticId  = "static";

  viewer->addParameter(parameter);
  if (parameter.dataset.encodedType != EncodeType::NONE) {
    primaryId = "dynamic";
    viewer->setColor(primaryId, 1.0, 0.0, 1.0);
    viewer->setColor(staticId, 1.0, 1.0, 1.0);
  }
  viewer->setPrimaryId(primaryId);

  auto datasetLoading = [&] {
    try {
      const DatasetReader::Ptr reader = newReader(parameter.reader, parameter.dataset);
      PreProcessor             preProcessor(parameter.preProcess);

      FramePtr     staticFrame;
      GroupOfFrame frames;
      GroupOfFrame staticFrames;
      const auto   framesMap         = jpcc::make_shared<GroupOfFrameMap>();
      const size_t groupOfFramesSize = parameter.groupOfFramesSize;
      size_t       startFrameNumber  = parameter.dataset.getStartFrameNumber();
      const size_t endFrameNumber    = parameter.dataset.getEndFrameNumber();

      if (parameter.dataset.encodedType == EncodeType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED) {
        staticFrame = jpcc::make_shared<Frame>();
      }

      while (run && startFrameNumber < endFrameNumber) {
        clock.start();
        switch (parameter.dataset.encodedType) {
          case EncodeType::NONE:
            reader->loadAll(startFrameNumber, groupOfFramesSize, frames, parameter.parallel);
            preProcessor.process(frames, framesMap, parameter.parallel);
            framesMap->insert_or_assign(primaryId, frames);
            break;
          case EncodeType::DYNAMIC_STATIC:
            reader->load(0, startFrameNumber, groupOfFramesSize, frames, parameter.parallel);
            reader->load(1, startFrameNumber, groupOfFramesSize, staticFrames, parameter.parallel);
            framesMap->insert_or_assign(primaryId, frames);
            framesMap->insert_or_assign(staticId, staticFrames);
            break;
          case EncodeType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED:
            GroupOfFrame staticAddedFrames;
            GroupOfFrame staticRemovedFrames;
            reader->load(0, startFrameNumber, groupOfFramesSize, frames, parameter.parallel);
            reader->load(1, startFrameNumber, groupOfFramesSize, staticAddedFrames, parameter.parallel);
            reader->load(2, startFrameNumber, groupOfFramesSize, staticRemovedFrames, parameter.parallel);
            staticFrames.clear();
            for (size_t i = 0; i < staticAddedFrames.size(); i++) {
              if (staticRemovedFrames.at(i)) {
                for (const auto& pointToRemove : staticRemovedFrames.at(i)->points) {
                  staticFrame->erase(remove_if(staticFrame->begin(), staticFrame->end(),
                                               [&pointToRemove](const auto& point) {
                                                 return point.x == pointToRemove.x && point.y == pointToRemove.y &&
                                                        point.z == pointToRemove.z;
                                               }),
                                     staticFrame->end());
                }
              }
              if (staticAddedFrames.at(i)) {
                staticFrame->insert(staticFrame->end(), staticAddedFrames.at(i)->points.begin(),
                                    staticAddedFrames.at(i)->points.end());
              }
              auto tmpFrame = jpcc::make_shared<Frame>();
              pcl::copyPointCloud(*staticFrame, *tmpFrame);
              staticFrames.push_back(tmpFrame);
            }
            framesMap->insert_or_assign(primaryId, frames);
            framesMap->insert_or_assign(staticId, staticFrames);
            break;
        }
        clock.stop();

        viewer->enqueue(*framesMap);

        while (run && viewer->isFull()) { this_thread::sleep_for(100ms); }

        if (frames.size() < groupOfFramesSize) { break; }

        startFrameNumber += groupOfFramesSize;
      }

      while (!viewer->isEmpty()) { this_thread::sleep_for(100ms); }

    } catch (exception& e) { cerr << e.what() << endl; }
    run = false;
  };

  thread datasetLoadingThread(datasetLoading);
  while (!viewer->wasStopped() && run) { viewer->spinOnce(1000); }
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

  cout << "JPCC App Dataset Viewer End" << endl;
  return 0;
}