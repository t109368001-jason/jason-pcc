#include <jpcc/metric/JPCCMetric.h>

#include <fstream>
#include <iomanip>
#include <iostream>

#include <boost/config.hpp>

using namespace std;
using namespace pcc;

namespace jpcc::metric {

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCMetric::JPCCMetric(const JPCCMetricParameter& parameter) :
    parameter_(parameter),
    frameNumberSet_(),
    pointsNameSet_(),
    bytesNameSet_(),
    c2cMSENameSet_(),
    c2cPSNRNameSet_(),
    c2pMSENameSet_(),
    c2pPSNRNameSet_(),
    clockNameSet_(),
    pointsMapMap_(),
    bytesMapMap_(),
    c2cMSEMapMap_(),
    c2cPSNRMapMap_(),
    c2pMSEMapMap_(),
    c2pPSNRMapMap_(),
    clockMapMap_() {}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCMetric::addBytes(const std::string& name, FrameNumber frameNumber, const uint64_t bytes) {
  cout << __FUNCTION__ << "() "
       << "name=" << name << ", "
       << "frameNumber=" << frameNumber << ", "
       << "bytes=" << bytes << std::endl;
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
  std::map<std::string, double>   c2cMSESumMap_;
  std::map<std::string, double>   c2cPSNRSumMap_;
  std::map<std::string, double>   c2pMSESumMap_;
  std::map<std::string, double>   c2pPSNRSumMap_;
  std::map<std::string, double>   clockSumMap_;

  std::ofstream metricCSV(parameter_.outputCSVFolderPath / "metric.csv");
  {  // write header
    metricCSV << "Frame Number,";
    for (const auto& name : pointsNameSet_) { metricCSV << name << " (points),"; }
    for (const auto& name : bytesNameSet_) { metricCSV << name << " (bytes),"; }
    for (const auto& name : c2cMSENameSet_) { metricCSV << name << " (mm^2),"; }
    for (const auto& name : c2cPSNRNameSet_) { metricCSV << name << " (db),"; }
    for (const auto& name : c2pMSENameSet_) { metricCSV << name << " (mm^2),"; }
    for (const auto& name : c2pPSNRNameSet_) { metricCSV << name << " (db),"; }
    for (const auto& name : clockNameSet_) { metricCSV << name << " (s),"; }
    metricCSV << std::endl;
  }
  for (const auto& frameNumber : frameNumberSet_) {  // write rows
    auto& pointsMap  = pointsMapMap_[frameNumber];
    auto& bytesMap   = bytesMapMap_[frameNumber];
    auto& c2cMSEMap  = c2cMSEMapMap_[frameNumber];
    auto& c2cPSNRMap = c2cPSNRMapMap_[frameNumber];
    auto& c2pMSEMap  = c2pMSEMapMap_[frameNumber];
    auto& c2pPSNRMap = c2pPSNRMapMap_[frameNumber];
    auto& clockMap   = clockMapMap_[frameNumber];
    metricCSV << frameNumber << ",";
    for (const auto& name : pointsNameSet_) {
      const auto& points = pointsMap[name];
      metricCSV << points << ",";
      pointsSumMap_[name] += points;
    }
    for (const auto& name : bytesNameSet_) {
      const auto& bytes = bytesMap[name];
      metricCSV << bytes << ",";
      bytesSumMap_[name] += bytes;
    }
    for (const auto& name : c2cMSENameSet_) {
      const auto& mse = c2cMSEMap[name];
      metricCSV << mse << ",";
      c2cMSESumMap_[name] += mse;
    }
    for (const auto& name : c2cPSNRNameSet_) {
      const auto& psnr = c2cPSNRMap[name];
      metricCSV << psnr << ",";
      c2cPSNRSumMap_[name] += psnr;
    }
    for (const auto& name : c2pMSENameSet_) {
      const auto& mse = c2pMSEMap[name];
      metricCSV << mse << ",";
      c2pMSESumMap_[name] += mse;
    }
    for (const auto& name : c2pPSNRNameSet_) {
      const auto& psnr = c2pPSNRMap[name];
      metricCSV << psnr << ",";
      c2pPSNRSumMap_[name] += psnr;
    }
    for (const auto& name : clockNameSet_) {
      double duration = (double)clockMap[name].count().count() / 1000000000.0;
      metricCSV << duration << ",";
      clockSumMap_[name] += duration;
    }
    metricCSV << std::endl;
  }
  std::ofstream metricSummaryCSV(parameter_.outputCSVFolderPath / "metric-summary.csv");
  metricSummaryCSV << "Platform," << BOOST_PLATFORM << endl;
#if defined(BOOST_CXX_VERSION)
  metricSummaryCSV << "C++ Version," << BOOST_CXX_VERSION << endl;
#elif defined(__cplusplus)
  metricSummaryCSV << "C++ Version," << __cplusplus << endl;
#endif
  metricSummaryCSV << "Compiler," << BOOST_COMPILER << endl;

  cout << "Environment Info:" << endl;
  cout << "  Platform          :" << BOOST_PLATFORM << endl;
#if defined(BOOST_CXX_VERSION)
  cout << "  C++ Version       :" << BOOST_CXX_VERSION << endl;
#elif defined(__cplusplus)
  cout << "  C++ Version       :" << __cplusplus << endl;
#endif
  cout << "  Compiler          :" << BOOST_COMPILER << endl;

  unsigned long frameCount = frameNumberSet_.size();
  metricSummaryCSV << "Frame Count," << frameCount << endl;

  int maxNameLength = 0;
  for (const auto& name : pointsNameSet_) { maxNameLength = max(maxNameLength, (int)name.size()); }
  for (const auto& name : bytesNameSet_) { maxNameLength = max(maxNameLength, (int)name.size()); }
  for (const auto& name : c2cMSENameSet_) { maxNameLength = max(maxNameLength, (int)name.size()); }
  for (const auto& name : c2cPSNRNameSet_) { maxNameLength = max(maxNameLength, (int)name.size()); }
  for (const auto& name : c2pMSENameSet_) { maxNameLength = max(maxNameLength, (int)name.size()); }
  for (const auto& name : c2pPSNRNameSet_) { maxNameLength = max(maxNameLength, (int)name.size()); }
  for (const auto& name : clockNameSet_) { maxNameLength = max(maxNameLength, (int)name.size()); }
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
  cout << "  MSE (c2c):" << endl;
  for (const auto& [name, mse] : c2cMSESumMap_) {  //
    metricSummaryCSV << name << " (mm^2)," << mse / (double)frameCount << endl;
    cout << "    " << std::setfill(' ') << std::setw(maxNameLength) << name << left << ": " << mse / (double)frameCount
         << " mm^2" << endl;
  }
  cout << "  PSNR (c2c):" << endl;
  for (const auto& [name, psnr] : c2cPSNRSumMap_) {  //
    metricSummaryCSV << name << " (db)," << psnr / (double)frameCount << endl;
    cout << "    " << std::setfill(' ') << std::setw(maxNameLength) << name << left << ": " << psnr / (double)frameCount
         << " db" << endl;
  }
  cout << "  MSE (c2p):" << endl;
  for (const auto& [name, mse] : c2pMSESumMap_) {  //
    metricSummaryCSV << name << " (mm^2)," << mse / (double)frameCount << endl;
    cout << "    " << std::setfill(' ') << std::setw(maxNameLength) << name << left << ": " << mse / (double)frameCount
         << " mm^2" << endl;
  }
  cout << "  PSNR (c2p):" << endl;
  for (const auto& [name, psnr] : c2pPSNRSumMap_) {  //
    metricSummaryCSV << name << " (db)," << psnr / (double)frameCount << endl;
    cout << "    " << std::setfill(' ') << std::setw(maxNameLength) << name << left << ": " << psnr / (double)frameCount
         << " db" << endl;
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