#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

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

using PointT = Point;

void main_(const AppParameter& parameter, StopwatchUserTime& clock) {
  const auto   viewer    = jpcc::make_shared<JPCCVisualizer<PointT>>(parameter.visualizerParameter);
  const string primaryId = "0";

  viewer->addParameter(parameter);
  viewer->setPrimaryId(primaryId);
  viewer->setColor("0", 1.0, 1.0, 1.0);
  viewer->setColor("1", 1.0, 0.0, 0.0);
  viewer->setColor("2", 0.0, 1.0, 0.0);
  viewer->setColor("3", 0.0, 0.0, 1.0);

  const DatasetReader<PointT>::Ptr reader = newReader<PointT>(parameter.reader, parameter.dataset);
  PreProcessor<PointT>             preProcessor(parameter.preProcess);

  GroupOfFrame<PointT> frames0;
  GroupOfFrame<PointT> frames1;
  GroupOfFrame<PointT> frames2;
  GroupOfFrame<PointT> frames3;
  const auto           framesMap = jpcc::make_shared<PreProcessor<PointT>::GroupOfFrameMap>();

  size_t startFrameNumber0 = parameter.dataset.getStartFrameNumber();
  size_t startFrameNumber1 = parameter.dataset.getStartFrameNumber();
  size_t startFrameNumber2 = parameter.dataset.getStartFrameNumber();
  size_t startFrameNumber3 = parameter.dataset.getStartFrameNumber();

  viewer->registerKeyboardEvent(
      [&](const pcl::visualization::KeyboardEvent& event) {
        if (!event.keyUp()) { return false; }
        if (event.getKeyCode() != ' ' && (event.getKeyCode() < '0' || event.getKeyCode() > '3')) { return false; }
        if (event.getKeyCode() == '0' || event.getKeyCode() == ' ') {
          cout << "0" << endl;
          clock.start();
          reader->load(0, startFrameNumber0++, 1, frames0);
          preProcessor.process(frames0, nullptr, parameter.parallel);
          clock.stop();
          framesMap->insert_or_assign("0", frames0);
        }
        if (event.getKeyCode() == '1' || event.getKeyCode() == ' ') {
          cout << "1" << endl;
          clock.start();
          reader->load(1, startFrameNumber1++, 1, frames1);
          preProcessor.process(frames1, nullptr, parameter.parallel);
          clock.stop();
          framesMap->insert_or_assign("1", frames1);
        }
        if (event.getKeyCode() == '2' || event.getKeyCode() == ' ') {
          cout << "2" << endl;
          clock.start();
          reader->load(2, startFrameNumber2++, 1, frames2);
          preProcessor.process(frames2, nullptr, parameter.parallel);
          clock.stop();
          framesMap->insert_or_assign("2", frames2);
        }
        if (event.getKeyCode() == '3' || event.getKeyCode() == ' ') {
          cout << "3" << endl;
          clock.start();
          reader->load(3, startFrameNumber3++, 1, frames3);
          preProcessor.process(frames3, nullptr, parameter.parallel);
          clock.stop();
          framesMap->insert_or_assign("3", frames3);
        }
        viewer->enqueue(*framesMap);
        viewer->nextFrame();
        return true;
      },
      "");

  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    // this_thread::sleep_for(100ms);
  }
}

int main(int argc, char* argv[]) {
  cout << "JPCC App Dataset Registration Tool Start" << endl;

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

  cout << "JPCC App Dataset Registration Tool End" << endl;
  return 0;
}