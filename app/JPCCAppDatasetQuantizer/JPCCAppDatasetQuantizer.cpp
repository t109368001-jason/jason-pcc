#include <chrono>
#include <iostream>
#include <vector>

#include <boost/log/trivial.hpp>

#include <jpcc/common/ParameterParser.h>
#include <jpcc/io/Reader.h>
#include <jpcc/io/PlyIO.h>

#include "AppParameter.h"

using namespace std;
using namespace std::chrono;
using namespace jpcc;
using namespace jpcc::io;

void quantize(const AppParameter& parameter, Stopwatch& clock) {
  const typename DatasetReader::Ptr reader = newReader(parameter.inputReader, parameter.inputDataset);

  GroupOfFrame frames;
  size_t       frameNumber    = parameter.inputDataset.getStartFrameNumber();
  const size_t endFrameNumber = parameter.inputDataset.getEndFrameNumber();
  while (frameNumber < endFrameNumber) {
    size_t groupOfFramesSize = std::min(parameter.groupOfFramesSize, endFrameNumber - frameNumber);
    clock.start();
    reader->loadAll(frameNumber, groupOfFramesSize, frames, parameter.parallel);
    clock.stop();

    for (auto& frame : frames) {
      for (size_t i = 0; i < frame->getPointCount(); i++) {
        auto& point = (*frame)[i];
        point[0] /= parameter.qp;
        point[1] /= parameter.qp;
        point[2] /= parameter.qp;
      }
    }

    savePly(frames, parameter.outputDataset.getFilePath(), parameter.parallel, false);

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

    clockWall.start();
    quantize(parameter, clockUser);
    clockWall.stop();

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