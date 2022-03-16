#include <chrono>
#include <iostream>
#include <vector>

#include <PCCChrono.h>
#include <PCCMemory.h>

#include <jpcc/common/ParameterParser.h>
#include <jpcc/io/DatasetReader.h>
#include <jpcc/io/PlyIO.h>

#include "AppParameter.h"

using namespace std::chrono;
using namespace pcc;
using namespace jpcc;
using namespace jpcc::io;

void parse(const AppParameter& appParameter, pcc::chrono::StopwatchUserTime& clock) {
  DatasetReader<>::Ptr reader =
      newReader<>(appParameter.inputDatasetReaderParameter, appParameter.inputDatasetParameter);
  GroupOfFrame<> frames;
  size_t         groupOfFramesSize = 32;
  size_t         startFrameNumber  = appParameter.inputDatasetParameter.getStartFrameNumbers();
  size_t         endFrameNumber    = startFrameNumber + appParameter.inputDatasetParameter.getFrameCounts();
  while (startFrameNumber < endFrameNumber) {
    clock.start();
    reader->loadAll(startFrameNumber, groupOfFramesSize, frames, appParameter.parallel);
    clock.stop();

    savePly(frames, appParameter.outputDatasetParameter.getFilePath(), appParameter.parallel);

    startFrameNumber += groupOfFramesSize;
  }
}

int main(int argc, char* argv[]) {
  std::cout << "JPCC App Dataset Parser Start" << std::endl;

  AppParameter appParameter;
  try {
    ParameterParser pp;
    pp.add(appParameter);
    pp.add(appParameter.inputDatasetParameter);
    pp.add(appParameter.inputDatasetReaderParameter);
    pp.add(appParameter.outputDatasetParameter);
    if (!pp.parse(argc, argv)) { return 1; }
    std::cout << appParameter.inputDatasetParameter << std::endl;
    std::cout << appParameter.inputDatasetReaderParameter << std::endl;
    std::cout << appParameter.outputDatasetParameter << std::endl;
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }

  try {
    // Timers to count elapsed wall/user time
    pcc::chrono::Stopwatch<steady_clock> clockWall;
    pcc::chrono::StopwatchUserTime       clockUser;

    clockWall.start();
    parse(appParameter, clockUser);
    clockWall.stop();

    auto totalWall      = duration_cast<milliseconds>(clockWall.count()).count();
    auto totalUserSelf  = duration_cast<milliseconds>(clockUser.self.count()).count();
    auto totalUserChild = duration_cast<milliseconds>(clockUser.children.count()).count();
    std::cout << "Processing time (wall): " << (float)totalWall / 1000.0 << " s\n";
    std::cout << "Processing time (user.self): " << (float)totalUserSelf / 1000.0 << " s\n";
    std::cout << "Processing time (user.children): " << (float)totalUserChild / 1000.0 << " s\n";
    std::cout << "Peak memory: " << getPeakMemory() << " KB\n";
  } catch (std::exception& e) { std::cerr << e.what() << std::endl; }

  std::cout << "JPCC App Dataset Parser End" << std::endl;
  return 0;
}