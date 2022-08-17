#include <jpcc/metric/JPCCMetric.h>

#include <iomanip>
#include <iostream>

using namespace std;
using namespace pcc;
using namespace pcc::chrono;

namespace jpcc::metric {

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCMetric::JPCCMetric() : clockMap_(), pointsMap_(), bytesMap_() {}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCMetric::start(const std::string& name) { clockMap_[name].start(); }

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCMetric::stop(const std::string& name) { clockMap_[name].stop(); }

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCMetric::addBytes(const std::string& name, const uint64_t bytes) { bytesMap_[name] = bytes; }

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCMetric::show() const {
  size_t maxKeyLength = 0;
  for (const auto& [name, _] : pointsMap_) { maxKeyLength = max(maxKeyLength, name.size()); }
  for (const auto& [name, _] : bytesMap_) { maxKeyLength = max(maxKeyLength, name.size()); }
  for (const auto& [name, _] : clockMap_) { maxKeyLength = max(maxKeyLength, name.size()); }
  cout << endl;
  cout << "Metric:" << endl;
  cout << "  Points:" << endl;
  for (const auto& [name, points] : pointsMap_) {  //
    cout << "    " << std::setfill(' ') << std::setw(maxKeyLength) << name << left << ": " << points << " points"
         << endl;
  }
  cout << "  Bytes:" << endl;
  for (const auto& [name, bytes] : bytesMap_) {  //
    cout << "    " << std::setfill(' ') << std::setw(maxKeyLength) << name << left << ": " << bytes << " bytes" << endl;
  }
  cout << "  Processing time:" << endl;
  for (const auto& [name, stopWatch] : clockMap_) {
    cout << "    " << std::setfill(' ') << std::setw(maxKeyLength) << name << left << ": "
         << (float)stopWatch.count().count() / 1000000000.0 << " s" << endl;
  }
  cout << "Peak memory: " << getPeakMemory() << " KB\n\n";
}

}  // namespace jpcc::metric