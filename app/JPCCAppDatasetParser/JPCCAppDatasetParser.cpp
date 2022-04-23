#include <chrono>
#include <iostream>
#include <vector>

#include <jpcc/common/ParameterParser.h>
#include <jpcc/io/DatasetReader.h>
#include <jpcc/io/PlyIO.h>

#include <PCCChrono.h>
#include <PCCMemory.h>

#include "AppParameter.h"

using namespace std;
using namespace std::chrono;
using namespace pcc;
using namespace pcc::chrono;
using namespace jpcc;
using namespace jpcc::io;

void parse(const AppParameter& parameter, StopwatchUserTime& clock) {
  const DatasetReader<>::Ptr reader = newReader<>(parameter.inputReader, parameter.inputDataset);
  GroupOfFrame<>             frames;
  const size_t               groupOfFramesSize = 32;
  size_t                     startFrameNumber  = parameter.inputDataset.getStartFrameNumber();
  const size_t               endFrameNumber    = startFrameNumber + parameter.inputDataset.getFrameCounts();
  while (startFrameNumber < endFrameNumber) {
    clock.start();
    reader->loadAll(startFrameNumber, groupOfFramesSize, frames, parameter.parallel);
    clock.stop();

    savePly(frames, parameter.outputDataset.getFilePath(), parameter.parallel);

    startFrameNumber += groupOfFramesSize;
  }
}

int main(int argc, char* argv[]) {
  cout << "JPCC App Dataset Parser Start" << endl;

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
    // Timers to count elapsed wall/user time
    Stopwatch<steady_clock> clockWall;
    StopwatchUserTime       clockUser;

    clockWall.start();
    parse(parameter, clockUser);
    clockWall.stop();

    auto totalWall      = duration_cast<milliseconds>(clockWall.count()).count();
    auto totalUserSelf  = duration_cast<milliseconds>(clockUser.self.count()).count();
    auto totalUserChild = duration_cast<milliseconds>(clockUser.children.count()).count();
    cout << "Processing time (wall): " << (float)totalWall / 1000.0 << " s\n";
    cout << "Processing time (user.self): " << (float)totalUserSelf / 1000.0 << " s\n";
    cout << "Processing time (user.children): " << (float)totalUserChild / 1000.0 << " s\n";
    cout << "Peak memory: " << getPeakMemory() << " KB\n";
  } catch (exception& e) { cerr << e.what() << endl; }

  cout << "JPCC App Dataset Parser End" << endl;
  return 0;
}