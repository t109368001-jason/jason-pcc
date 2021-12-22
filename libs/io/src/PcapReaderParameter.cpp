#include <jpcc/io/PcapReaderParameter.h>

#include <exception>

namespace jpcc {
namespace io {

using namespace std;
using namespace po;

#define PCAP_READER_OPT_PREFIX "pcapReader"
#define POINT_TYPES_OPT PCAP_READER_OPT_PREFIX ".pointTypes"
#define EPSILON_OPT PCAP_READER_OPT_PREFIX ".epsilon"
#define USE_RADIAN_OPT PCAP_READER_OPT_PREFIX ".useRadian"

PcapReaderParameter::PcapReaderParameter() : pointTypes(), epsilon(0.001), useRadian(true) {}

options_description PcapReaderParameter::getOpts() {
  options_description opts("DatasetOptions");
  opts.add_options()                                                                     //
      (POINT_TYPES_OPT,                                                                  //
       value<vector<string>>(&pointTypes_)->required(),                                  //
       "types to read: [xyz, intensity, azimuth, vertical, distance, id, time]")         //
      (EPSILON_OPT,                                                                      //
       value<float>(&epsilon)->default_value(epsilon),                                   //
       "epsilon, drop laser point if distance < epsilon")                                //
      (USE_RADIAN_OPT, value<bool>(&useRadian)->default_value(useRadian), "use radian")  //
      ;
  return opts;
}

void PcapReaderParameter::notify() {
  assert(epsilon >= 0);
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
  out << "\t" EPSILON_OPT "=" << obj.epsilon << endl;
  out << "\t" USE_RADIAN_OPT "=" << obj.useRadian << endl;
  return out;
}

}  // namespace io
}  // namespace jpcc
