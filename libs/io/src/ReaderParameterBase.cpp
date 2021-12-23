#include <jpcc/io/ReaderParameterBase.h>

#include <utility>

namespace jpcc::io {

using namespace std;
using namespace po;

#define POINT_TYPES_OPT ".pointTypes"
#define EPSILON_OPT ".epsilon"

ReaderParameterBase::ReaderParameterBase(string prefix, const string& caption) :
    Parameter(std::move(prefix), caption), pointTypes(), epsilon(0.001) {
  opts_.add_options()                                                             //
      ((string(prefix_ + POINT_TYPES_OPT)).c_str(),                               //
       value<vector<string>>(&pointTypes_),                                       //
       "types to read: [xyz, intensity, azimuth, vertical, distance, id, time]")  //
      ((string(prefix_ + EPSILON_OPT)).c_str(),                                   //
       value<float>(&epsilon)->default_value(epsilon),                            //
       "epsilon, drop laser point if distance < epsilon")                         //
      ;
}

void ReaderParameterBase::notify() {
  assert(epsilon >= 0);
  assert(!pointTypes_.empty());
  for (string& pointType : pointTypes_) { pointTypes.insert(pointType); }
}

ostream& operator<<(ostream& out, const ReaderParameterBase& obj) {
  out << obj.caption_ << endl;
  out << "\t" << obj.prefix_ << POINT_TYPES_OPT "=";
  size_t                i;
  set<string>::iterator it;
  for (i = 0, it = obj.pointTypes.begin(); i < obj.pointTypes.size(); i++, it++) {
    if (i == 0) { out << "["; }
    if (i != 0) { out << ", "; }
    out << *it;
    if (i == (obj.pointTypes.size() - 1)) { out << "]"; }
  }
  out << endl;
  out << "\t" << obj.prefix_ << EPSILON_OPT "=" << obj.epsilon << endl;
  return out;
}

}  // namespace jpcc::io
