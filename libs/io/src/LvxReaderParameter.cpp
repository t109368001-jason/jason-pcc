#include <jpcc/io/LvxReaderParameter.h>

#include <exception>

namespace jpcc {
namespace io {

using namespace std;
using namespace po;

#define LVX_READER_OPT_PREFIX "lvxReader"
#define POINT_TYPES_OPT LVX_READER_OPT_PREFIX ".pointTypes"
#define EPSILON_OPT LVX_READER_OPT_PREFIX ".epsilon"
#define FREQUENCY_OPT LVX_READER_OPT_PREFIX ".frequency"

LvxReaderParameter::LvxReaderParameter() : pointTypes(), epsilon(0.001), frequency(10.0) {}

options_description LvxReaderParameter::getOpts() {
  options_description opts("DatasetOptions");
  opts.add_options()                                                              //
      (POINT_TYPES_OPT,                                                           //
       value<vector<string>>(&pointTypes_)->required(),                           //
       "types to read: [xyz, intensity, azimuth, vertical, distance, id, time]")  //
      (EPSILON_OPT,                                                               //
       value<float>(&epsilon)->default_value(epsilon),                            //
       "epsilon, drop laser point if distance < epsilon")                         //
      (FREQUENCY_OPT,                                                             //
       value<float>(&frequency)->default_value(frequency),                        //
       "frame frequency")                                                         //
      ;
  return opts;
}

void LvxReaderParameter::notify() {
  assert(epsilon >= 0);
  assert(frequency > 0);
  for (string& pointType : pointTypes_) { pointTypes.insert(pointType); }
  interval = 1000 / frequency;
}

ostream& operator<<(ostream& out, const LvxReaderParameter& obj) {
  out << "LvxReaderParameter" << endl;
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
  out << "\t" FREQUENCY_OPT "=" << obj.frequency << endl;
  return out;
}

}  // namespace io
}  // namespace jpcc
