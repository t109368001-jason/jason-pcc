#include <chrono>
#include <iostream>
#include <mutex>
#include <vector>

#include <boost/log/trivial.hpp>

#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

#include <jpcc/common/ParameterParser.h>
#include <jpcc/io/Reader.h>
#include <jpcc/process/PreProcessor.h>
#include <jpcc/visualization/JPCCVisualizer.h>

#include "AppParameter.h"

using namespace std;
using namespace std::chrono;
using namespace jpcc;
using namespace jpcc::io;
using namespace jpcc::process;
using namespace jpcc::visualization;

using PointT = pcl::PointXYZ;

void main_(const AppParameter& parameter, Stopwatch& clock) {
  const auto   viewer    = jpcc::make_shared<JPCCVisualizer<PointT>>(parameter.visualizerParameter);
  const string primaryId = "0";

  viewer->addParameter(parameter);
  viewer->setPrimaryId(primaryId);
  viewer->setColor("0", 1.0, 1.0, 1.0);
  viewer->setColor("1", 1.0, 0.0, 0.0);
  viewer->setColor("2", 0.0, 1.0, 0.0);
  viewer->setColor("3", 0.0, 0.0, 1.0);

  const DatasetReader::Ptr reader = newReader(parameter.reader, parameter.dataset);
  PreProcessor             preProcessor(parameter.preProcess);

  pcl::Registration<PointT, PointT>::Ptr registration;

  if (parameter.registration == "icp") {
    auto icp     = pcl::make_shared<pcl::IterativeClosestPoint<PointT, PointT>>();
    registration = icp;
  }
  if (parameter.registration == "ndt") {
    auto ndt = pcl::make_shared<pcl::NormalDistributionsTransform<PointT, PointT>>();
    ndt->setResolution(100.0);
    registration = ndt;
  }

  GroupOfPclFrameMap<PointT>    pclFramesMap = {{"0", GroupOfPclFrame<PointT>{make_shared<PclFrame<PointT>>()}},
                                                {"1", GroupOfPclFrame<PointT>{make_shared<PclFrame<PointT>>()}},
                                                {"2", GroupOfPclFrame<PointT>{make_shared<PclFrame<PointT>>()}},
                                                {"3", GroupOfPclFrame<PointT>{make_shared<PclFrame<PointT>>()}}};
  std::map<std::string, size_t> startFrameNumberMap{{"0", parameter.dataset.getStartFrameNumber()},
                                                    {"1", parameter.dataset.getStartFrameNumber()},
                                                    {"2", parameter.dataset.getStartFrameNumber()},
                                                    {"3", parameter.dataset.getStartFrameNumber()}};

  viewer->registerKeyboardEvent(
      [&](const pcl::visualization::KeyboardEvent& event) {
        if (!event.keyUp()) { return false; }
        if (event.getKeyCode() != ' ' && (event.getKeyCode() < '0' || event.getKeyCode() > '3')) { return false; }
        auto handleKey = [&](const uint8_t& keyCode) {
          std::string id(1, (char)keyCode);
          auto&       pclFrames        = pclFramesMap[id];
          auto&       startFrameNumber = startFrameNumberMap[id];
          BOOST_LOG_TRIVIAL(info) << id;
          clock.start();
          GroupOfFrame frames;
          reader->load(keyCode - '0', startFrameNumber++, 1, frames, parameter.parallel);
          preProcessor.process(frames, nullptr, parameter.parallel);
          frames[0]->toPcl<PointT>(pclFrames[0]);
          clock.stop();
          if (registration) {
            if (event.getKeyCode() == '0') {
              registration->setInputTarget(pclFrames[0]);
            } else {
              auto frame = jpcc::make_shared<PclFrame<PointT>>();

              registration->setInputSource(pclFrames[0]);
              registration->align(*frame);
              BOOST_LOG_TRIVIAL(info) << registration->getFinalTransformation();
              pclFrames[0] = frame;
            }
          }
        };
        if (event.getKeyCode() >= '0' && event.getKeyCode() <= '3') {
          handleKey(event.getKeyCode());
        } else if (event.getKeyCode() == ' ') {
          handleKey('0');
          handleKey('1');
          handleKey('2');
          handleKey('3');
        }
        BOOST_LOG_TRIVIAL(info) << startFrameNumberMap["0"] << ", "  //
                                << startFrameNumberMap["1"] << ", "  //
                                << startFrameNumberMap["2"] << ", "  //
                                << startFrameNumberMap["3"];
        viewer->enqueue(pclFramesMap);
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
  BOOST_LOG_TRIVIAL(info) << "JPCC App Dataset Registration Tool Start";

  AppParameter parameter;
  try {
    ParameterParser pp;
    pp.add(parameter);
    if (!pp.parse(argc, argv)) { return 1; }
    BOOST_LOG_TRIVIAL(info) << parameter;
  } catch (exception& e) {
    BOOST_LOG_TRIVIAL(error) << e.what();
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
    BOOST_LOG_TRIVIAL(info) << "Processing time (wall): " << static_cast<float>(totalWall) / 1000.0 << " s";
    BOOST_LOG_TRIVIAL(info) << "Processing time (user): " << static_cast<float>(totalUser) / 1000.0 << " s";
    BOOST_LOG_TRIVIAL(info) << "Peak memory: " << getPeakMemory() << " KB";
  } catch (exception& e) { BOOST_LOG_TRIVIAL(error) << e.what(); }

  BOOST_LOG_TRIVIAL(info) << "JPCC App Dataset Registration Tool End";
  return 0;
}