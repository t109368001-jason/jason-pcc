#include <jpcc/io/LvxReaderParameter.h>

namespace jpcc::io {

using namespace std;
using namespace po;

#define LVX_READER_OPT_PREFIX "lvxReader"
#define FREQUENCY_OPT ".frequency"

LvxReaderParameter::LvxReaderParameter() : LvxReaderParameter(LVX_READER_OPT_PREFIX, "LvxReaderParameter") {}

LvxReaderParameter::LvxReaderParameter(const std::string& prefix, const std::string& caption) :
    Parameter(prefix, caption), frequency(10.0), interval(100.0) {
  opts_.add_options()                                       //
      (string(prefix + FREQUENCY_OPT).c_str(),              //
       value<float>(&frequency)->default_value(frequency),  //
       "frame frequency")                                   //
      ;
}

void LvxReaderParameter::notify() {
  assert(frequency > 0);
  interval = 1000 / frequency;
}

ostream& operator<<(ostream& out, const LvxReaderParameter& obj) {
  out << "\t" << obj.prefix_ << FREQUENCY_OPT "=" << obj.frequency << endl;
  out << "\tinterval=" << obj.interval << endl;
  return out;
}

}  // namespace jpcc::io
