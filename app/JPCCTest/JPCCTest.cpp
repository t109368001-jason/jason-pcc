#include <chrono>
#include <execution>
#include <filesystem>
#include <iostream>
#include <vector>

#include <jpcc/common/Common.h>
#include <jpcc/common/ParameterParser.h>
#include <jpcc/io/DatasetParameter.h>
#include <jpcc/io/PcapReaderParameter.h>
#include <jpcc/io/PcapReader.h>

#include <PCCChrono.h>
#include <PCCMemory.h>

using namespace pcc;
using namespace jpcc;
using namespace jpcc::common;
using namespace jpcc::io;

void test(const DatasetParameter&         datasetParameter,
          const PcapReaderParameter&      pcapReaderParameter,
          pcc::chrono::StopwatchUserTime& clock) {
  PcapReader                pcapReader(datasetParameter, pcapReaderParameter);
  std::vector<GroupOfFrame> sources;
  size_t                    groupOfFrameSize = 32;
  size_t                    startFrameIndex  = 0;
  while (true) {
    clock.start();
    pcapReader.loadAll(sources, startFrameIndex, groupOfFrameSize, false);
    clock.stop();
    if (std::all_of(sources.begin(), sources.end(), [&](auto& frames) { return frames.size() < groupOfFrameSize; })) {
      break;
    }
    startFrameIndex += groupOfFrameSize;
  }
}

int main(int argc, char* argv[]) {
  std::cout << "JPCC Test App Start" << std::endl;

  DatasetParameter    datasetParameter;
  PcapReaderParameter pcapReaderParameter;
  try {
    ParameterParser pp;
    pp.add(datasetParameter);
    pp.add(pcapReaderParameter);
    pp.parse(argc, argv);
    std::cout << datasetParameter << std::endl;
    std::cout << pcapReaderParameter << std::endl;
  } catch (std::exception& e) { std::cerr << e.what() << std::endl; }

  try {
    ParameterParser pp;
    // Timers to count elapsed wall/user time
    pcc::chrono::Stopwatch<std::chrono::steady_clock> clockWall;
    pcc::chrono::StopwatchUserTime                    clockUser;

    clockWall.start();
    test(datasetParameter, pcapReaderParameter, clockUser);
    clockWall.stop();

    auto totalWall      = std::chrono::duration_cast<std::chrono::milliseconds>(clockWall.count()).count();
    auto totalUserSelf  = std::chrono::duration_cast<std::chrono::milliseconds>(clockUser.self.count()).count();
    auto totalUserChild = std::chrono::duration_cast<std::chrono::milliseconds>(clockUser.children.count()).count();
    std::cout << "Processing time (wall): " << totalWall / 1000.0 << " s\n";
    std::cout << "Processing time (user.self): " << totalUserSelf / 1000.0 << " s\n";
    std::cout << "Processing time (user.children): " << totalUserChild / 1000.0 << " s\n";
    std::cout << "Peak memory: " << getPeakMemory() << " KB\n";
  } catch (std::exception& e) { std::cerr << e.what() << std::endl; }

  std::cout << "JPCC Test App End" << std::endl;
  return 0;
}