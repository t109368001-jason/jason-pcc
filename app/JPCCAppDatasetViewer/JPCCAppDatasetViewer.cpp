#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

#include <pcl/io/ply_io.h>

#include <jpcc/common/ParameterParser.h>
#include <jpcc/io/Reader.h>
#include <jpcc/process/PreProcessor.h>
#include <jpcc/process/OctreePointCloudOperation.h>
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
  const auto                       viewer = jpcc::make_shared<JPCCVisualizer>(parameter.visualizerParameter);
  OctreePointCloudOperation<>::Ptr octreePointCloudOperation;

  FramePtr background;
  if (!parameter.backgroundPath.empty() && std::filesystem::exists(parameter.backgroundPath)) {
    background = jpcc::make_shared<Frame>();
    pcl::io::loadPLYFile(parameter.backgroundPath, *background);
    octreePointCloudOperation = jpcc::make_shared<OctreePointCloudOperation<>>(0.1);
    octreePointCloudOperation->setSource(background);
  }

  atomic_bool  run(true);
  const string primaryId    = "cloud";
  const string backgroundId = "background";

  viewer->addParameter(parameter);
  viewer->setPrimaryId(primaryId);
  if (octreePointCloudOperation) { viewer->setColor(backgroundId, 1.0, 1.0, 1.0); }

  auto datasetLoading = [&] {
    try {
      const DatasetReader::Ptr reader = newReader(parameter.reader, parameter.dataset);
      PreProcessor             preProcessor(parameter.preProcess);

      GroupOfFrame frames;
      const auto   framesMap         = jpcc::make_shared<PreProcessor::GroupOfFrameMap>();
      const size_t groupOfFramesSize = 32;
      size_t       startFrameNumber  = parameter.dataset.getStartFrameNumber();
      reader->loadAll(startFrameNumber, 1, frames, parameter.parallel);
      startFrameNumber++;
      preProcessor.process(frames, framesMap, parameter.parallel);
      if (octreePointCloudOperation) {
        framesMap->insert_or_assign(backgroundId, GroupOfFrame{background});
        octreePointCloudOperation->setTarget(frames.at(0));
        frames.at(0) = octreePointCloudOperation->targetAndNotSource();
      }
      framesMap->insert_or_assign(primaryId, frames);
      viewer->enqueue(*framesMap);
      viewer->nextFrame();
      while (run) {
        clock.start();
        reader->loadAll(startFrameNumber, groupOfFramesSize, frames, parameter.parallel);
        preProcessor.process(frames, framesMap, parameter.parallel);
        clock.stop();

        while (run && viewer->isFull()) { this_thread::sleep_for(100ms); }

        if (octreePointCloudOperation) {
          GroupOfFrame backgrounds(frames.size());
          fill(backgrounds.begin(), backgrounds.end(), background);
          framesMap->insert_or_assign(backgroundId, backgrounds);
          for_each(frames.begin(), frames.end(), [&](auto& frame) {
            octreePointCloudOperation->setTarget(frame);
            frame = octreePointCloudOperation->targetAndNotSource();
          });
        }
        framesMap->insert_or_assign(primaryId, frames);
        viewer->enqueue(*framesMap);

        if (frames.size() < groupOfFramesSize) {
          startFrameNumber = 0;
          continue;
        }
        startFrameNumber += groupOfFramesSize;
      }
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