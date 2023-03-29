#include <chrono>
#include <iostream>
#include <mutex>
#include <vector>

#include <boost/log/trivial.hpp>

#include <jpcc/common/ParameterParser.h>
#include <jpcc/io/Reader.h>
#include <jpcc/process/Process.h>
#include <jpcc/process/PreProcessor.h>
#include <jpcc/visualization/JPCCVisualizer.h>

#include "AppParameter.h"

#define PCL_NO_PRECOMPILE
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

using namespace std;
using namespace std::chrono;
using namespace jpcc;
using namespace jpcc::io;
using namespace jpcc::process;
using namespace jpcc::visualization;

using PointT = pcl::PointXYZ;

void main_(const AppParameter& parameter, Stopwatch& clock) {
  JPCCVisualizer<PointT>::Ptr viewer;
  if (!parameter.headless) {
    viewer = jpcc::make_shared<JPCCVisualizer<PointT>>(parameter.visualizerParameter);
  }

  const string cloudPlaneId = "cloudPlane";
  const string cloudOtherId = "cloudOther";

  if (viewer) {
    viewer->addParameter(parameter);
    viewer->setPrimaryId(cloudOtherId);
    viewer->setColor(cloudOtherId, "z");
    viewer->setColor(cloudPlaneId, 1.0, 0.0, 1.0);
  }

  GroupOfPclFrameMap<PointT> framesMap;

  {
    const DatasetReader::Ptr reader = newReader(parameter.reader, parameter.dataset);
    PreProcessor             preProcessor(parameter.preProcess);

    GroupOfFrame frames;
    clock.start();
    reader->loadAll(parameter.dataset.getStartFrameNumber(), 1, frames, parameter.parallel);
    preProcessor.process(frames, nullptr, parameter.parallel);
    clock.stop();

    auto modelPlane = jpcc::make_shared<pcl::SampleConsensusModelPlane<PointT>>(frames.front()->toPcl<PointT>(), true);
    Indices                            indices;
    pcl::RandomSampleConsensus<PointT> ransac(modelPlane);
    ransac.setDistanceThreshold(parameter.distanceThreshold);
    ransac.computeModel();
    ransac.getInliers(indices);

    BOOST_LOG_TRIVIAL(info) << "RANSAC iteration=" << ransac.iterations_;
    BOOST_LOG_TRIVIAL(info) << "RANSAC coefficients=" << endl << ransac.model_coefficients_ << endl;

    if (!viewer) {
      return;
    }

    auto framePlane = jpcc::make_shared<Frame>();
    auto frameOther = jpcc::make_shared<Frame>();

    process::split(frames.front(), indices, framePlane, frameOther);

    framesMap.insert_or_assign(cloudPlaneId, GroupOfPclFrame<PointT>{framePlane->toPcl<PointT>()});
    framesMap.insert_or_assign(cloudOtherId, GroupOfPclFrame<PointT>{frameOther->toPcl<PointT>()});
  }
  viewer->enqueue(framesMap);
  viewer->nextFrame();

  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    // this_thread::sleep_for(100ms);
  }
}

int main(int argc, char* argv[]) {
  BOOST_LOG_TRIVIAL(info) << "JPCC App Ground Segmentation Start";

  AppParameter parameter;
  try {
    ParameterParser pp;
    pp.add(parameter);
    if (!pp.parse(argc, argv)) {
      return 1;
    }
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
  } catch (exception& e) {
    BOOST_LOG_TRIVIAL(error) << e.what();
  }

  BOOST_LOG_TRIVIAL(info) << "JPCC App Ground Segmentation End";
  return 0;
}