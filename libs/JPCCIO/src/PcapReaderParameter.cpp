#include <jpcc/io/PcapReaderParameter.h>

#include <exception>

namespace jpcc {
namespace io {

using namespace std;
using namespace po;

#define PCAP_READER_OPT_PREFIX "pcapReader"
#define POINT_TYPES_OPT PCAP_READER_OPT_PREFIX ".pointTypes"

PcapReaderParameter::PcapReaderParameter() : pointTypes() {}

options_description PcapReaderParameter::getOpts() {
  options_description opts("DatasetOptions");
  opts.add_options()                                                              //
      (POINT_TYPES_OPT,                                                           //
       value<vector<string>>(&pointTypes_)->required(),                           //
       "types to read: [azimuth, vertical, distance, intensity, id, time, xyz]")  //
      ;
  return opts;
}

void PcapReaderParameter::notify() {
  for (string& pointType : pointTypes_) { pointTypes.insert(pointType); }
}

ostream& operator<<(ostream& out, const PcapReaderParameter& obj) {
  out << "PcapReaderParameter" << endl;
  out << "\t" POINT_TYPES_OPT "=";
  size_t                i = 0;
  set<string>::iterator it;
  for (i = 0, it = obj.pointTypes.begin(); i < obj.pointTypes.size(); i++, it++) {
    if (i == 0) { out << "["; }
    if (i != 0) { out << ", "; }
    out << *it;
    if (i == (obj.pointTypes.size() - 1)) { out << "]" << endl; }
  }
  return out;
}

}  // namespace io
}  // namespace jpcc
