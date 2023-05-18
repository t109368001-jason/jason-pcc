#include <chrono>
#include <execution>
#include <filesystem>
#include <iostream>
#include <vector>

#include <boost/log/trivial.hpp>
#include <boost/range/counting_range.hpp>

#include <jpcc/common/ParameterParser.h>
#include <jpcc/io/Reader.h>
#include <jpcc/io/PlyIO.h>
#include <jpcc/metric/JPCCMetric.h>

#include "AppParameter.h"

using namespace std;
using namespace std::chrono;
using namespace jpcc;
using namespace jpcc::io;
using namespace jpcc::metric;

void runMetric(const AppParameter& parameter, Stopwatch& clock, JPCCMetric& metric) {
  const typename DatasetReader::Ptr reader            = newReader(parameter.inputReader, parameter.inputDataset);
  const typename DatasetReader::Ptr reconstructReader = newReader(parameter.inputReader, parameter.reconstructDataset);

  GroupOfFrame frames;
  GroupOfFrame reconstructFrames;
  size_t       frameNumber    = parameter.inputDataset.getStartFrameNumber();
  const size_t endFrameNumber = parameter.inputDataset.getEndFrameNumber();
  while (frameNumber < endFrameNumber) {
    size_t groupOfFramesSize = std::min(parameter.groupOfFramesSize, endFrameNumber - frameNumber);
    clock.start();

    reader->loadAll(frameNumber, groupOfFramesSize, frames, parameter.parallel);
    reconstructReader->loadAll(frameNumber, groupOfFramesSize, reconstructFrames, parameter.parallel);

    // copy normals
    metric.addPSNR("A2B", frames, reconstructFrames, parameter.parallel);

    clock.stop();
    frameNumber += groupOfFramesSize;
  }
}

int main(int argc, char* argv[]) {
  BOOST_LOG_TRIVIAL(info) << "JPCC App Dataset Quantizer Start";

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
    // Timers to count elapsed wall/user time
    Stopwatch clockWall;
    Stopwatch clockUser;

    // Timers to count elapsed wall/user time
    JPCCMetricParameter metricParameter;
    metricParameter.maximumValue = parameter.maximumValue;
    metricParameter.outputCSVFolderPath =
        filesystem::path(parameter.reconstructDataset.folderPrefix) / parameter.reconstructDataset.folder;
    JPCCMetric metric(metricParameter);

    clockWall.start();
    runMetric(parameter, clockUser, metric);
    clockWall.stop();

    metric.writeAndShow();

    auto totalWall = duration_cast<milliseconds>(clockWall.count()).count();
    auto totalUser = duration_cast<milliseconds>(clockUser.count()).count();
    BOOST_LOG_TRIVIAL(info) << "Processing time (wall): " << static_cast<float>(totalWall) / 1000.0 << " s";
    BOOST_LOG_TRIVIAL(info) << "Processing time (user): " << static_cast<float>(totalUser) / 1000.0 << " s";
    BOOST_LOG_TRIVIAL(info) << "Peak memory: " << getPeakMemory() << " KB";
  } catch (exception& e) {
    BOOST_LOG_TRIVIAL(error) << e.what();
  }

  BOOST_LOG_TRIVIAL(info) << "JPCC App Dataset Quantizer End";
  return 0;
}