#include <jpcc/metric/JPCCMetric.h>

#include <fstream>
#include <iomanip>
#include <iostream>

using namespace std;
using namespace pcc;
using namespace pcc::chrono;

namespace jpcc::metric {

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCMetric::JPCCMetric(const JPCCMetricParameter& parameter) :
    parameter_(parameter),
    frameNumberSet_(),
    pointsNameSet_(),
    bytesNameSet_(),
    clockNameSet_(),
    clockMapMap_(),
    pointsMapMap_(),
    bytesMapMap_() {}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCMetric::addBytes(const std::string& name, FrameNumber frameNumber, const uint64_t bytes) {
  frameNumberSet_.insert(frameNumber);
  bytesNameSet_.insert(name);
  bytesMapMap_[frameNumber][name] = bytes;
}

//////////////////////////////////////////////////////////////////////////////////////////////
ScopeStopwatch JPCCMetric::start(const std::string& name, const FrameNumber frameNumber) {
  frameNumberSet_.insert(frameNumber);
  clockNameSet_.insert(name);
  Stopwatch& clock = clockMapMap_[frameNumber][name];
  return {clock};
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCMetric::writeAndShow() {
  std::map<std::string, uint64_t> pointsSumMap_;
  std::map<std::string, uint64_t> bytesSumMap_;
  std::map<std::string, double>   clockSumMap_;

  std::ofstream metricCSV(parameter_.outputCSVFolderPath / "metric.csv");
  {  // write header
    for (const auto& name : pointsNameSet_) { metricCSV << name << " (points),"; }
    for (const auto& name : bytesNameSet_) { metricCSV << name << " (bytes),"; }
    for (const auto& name : clockNameSet_) { metricCSV << name << " (s),"; }
    metricCSV << std::endl;
  }
  for (const auto& frameNumber : frameNumberSet_) {  // write rows
    auto& pointsMap = pointsMapMap_[frameNumber];
    auto& bytesMap  = bytesMapMap_[frameNumber];
    auto& clockMap  = clockMapMap_[frameNumber];
    for (const auto& name : pointsNameSet_) {
      const uint64_t& points = pointsMap[name];
      metricCSV << points << ",";
      pointsSumMap_[name] += points;
    }
    for (const auto& name : bytesNameSet_) {
      const uint64_t& bytes = bytesMap[name];
      metricCSV << bytes << ",";
      bytesSumMap_[name] += bytes;
    }
    for (const auto& name : clockNameSet_) {
      double duration = (double)clockMap[name].count().count() / 1000000000.0;
      metricCSV << duration << ",";
      clockSumMap_[name] += duration;
    }
    metricCSV << std::endl;
  }
  std::ofstream metricSummaryCSV(parameter_.outputCSVFolderPath / "metric-summary.csv");
  metricSummaryCSV << "Frame Count," << frameNumberSet_.size() << endl;

  size_t maxNameLength = 0;
  for (const auto& name : pointsNameSet_) { maxNameLength = max(maxNameLength, name.size()); }
  for (const auto& name : bytesNameSet_) { maxNameLength = max(maxNameLength, name.size()); }
  for (const auto& name : clockNameSet_) { maxNameLength = max(maxNameLength, name.size()); }
  cout << endl;
  cout << "Metric:" << endl;
  cout << "  Points:" << endl;
  for (const auto& [name, points] : pointsSumMap_) {  //
    metricSummaryCSV << name << " (points)," << points << endl;
    cout << "    " << std::setfill(' ') << std::setw(maxNameLength) << name << left << ": " << points << " points"
         << endl;
  }
  cout << "  Bytes:" << endl;
  for (const auto& [name, bytes] : bytesSumMap_) {  //
    metricSummaryCSV << name << " (bytes)," << bytes << endl;
    cout << "    " << std::setfill(' ') << std::setw(maxNameLength) << name << left << ": " << bytes << " bytes"
         << endl;
  }
  cout << "  Processing time:" << endl;
  for (const auto& [name, duration] : clockSumMap_) {
    metricSummaryCSV << name << " (s)," << duration << endl;
    cout << "    " << std::setfill(' ') << std::setw(maxNameLength) << name << left << ": " << duration << " s" << endl;
  }
  metricSummaryCSV << "Peek memory (KB)," << getPeakMemory() << endl;
  cout << "Peak memory: " << getPeakMemory() << " KB\n\n";
}

}  // namespace jpcc::metric