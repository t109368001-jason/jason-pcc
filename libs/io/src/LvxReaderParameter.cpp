#include <jpcc/io/LvxReaderParameter.h>

namespace jpcc::io {

using namespace std;
using namespace po;

#define LVX_READER_OPT_PREFIX "lvxReader"
#define FREQUENCY_OPT ".frequency"

LvxReaderParameter::LvxReaderParameter() : LvxReaderParameter(LVX_READER_OPT_PREFIX, __FUNCTION__) {}

LvxReaderParameter::LvxReaderParameter(const string& prefix, const string& caption) :
    Parameter(prefix, caption), frequency(10.0), interval(100.0) {
  opts_.add_options()                                       //
      (string(prefix + FREQUENCY_OPT).c_str(),              //
       value<float>(&frequency)->default_value(frequency),  //
       "frame frequency")                                   //
      ;
}

void LvxReaderParameter::getShowTexts(vector<std::string>& showTexts) const {
  showTexts.push_back(prefix_ + FREQUENCY_OPT ": " + to_string(frequency));
}

void LvxReaderParameter::notify() {
  assert(frequency > 0);
  interval = 1000 / frequency;
}

ostream& operator<<(ostream& out, const LvxReaderParameter& obj) {
  obj.coutParameters(out)             //
      (FREQUENCY_OPT, obj.frequency)  //
      ("interval", obj.interval)      //
      ;
  return out;
}

}  // namespace jpcc::io
