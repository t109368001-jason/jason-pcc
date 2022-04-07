#include <jpcc/process/JPCCConditionalRemovalParameter.h>

namespace jpcc::process {

using namespace std;
using namespace po;

#define ENABLE_OPT ".enable"
#define CONDITIONS_OPT ".conditions"

JPCCConditionalRemovalParameter::JPCCConditionalRemovalParameter() :
    JPCCConditionalRemovalParameter(JPCC_CONDITIONAL_REMOVAL_OPT_PREFIX, __FUNCTION__) {}

JPCCConditionalRemovalParameter::JPCCConditionalRemovalParameter(const std::string& prefix,
                                                                 const std::string& caption) :
    Parameter(prefix, caption), conditions_(), enable(false), conditions() {
  opts_.add_options()                             //
      (string(prefix_ + ENABLE_OPT).c_str(),      //
       value<bool>(&enable),                      //
       "jpccConditionalRemoval enable")           //
      (string(prefix_ + CONDITIONS_OPT).c_str(),  //
       value<vector<string>>(&conditions_),       //
       "jpccConditionalRemoval conditions")       //
      ;
}

void JPCCConditionalRemovalParameter::notify() {
  if (enable) {
    conditions.resize(conditions_.size());
    std::transform(conditions_.begin(), conditions_.end(), conditions.begin(),
                   [](const string& condition) { return Condition(condition); });
  }
}

ostream& operator<<(ostream& out, const JPCCConditionalRemovalParameter& obj) {
  out << obj.caption_ << endl;
  out << "\t" << obj.prefix_ << ENABLE_OPT "=" << obj.enable << endl;
  out << "\t" << obj.prefix_ << CONDITIONS_OPT "=";
  for (size_t i = 0; i < obj.conditions_.size(); i++) {
    if (i == 0) { out << "["; }
    if (i != 0) { out << ", "; }
    out << "\"" << obj.conditions_.at(i) << "\"";
    if (i == (obj.conditions_.size() - 1)) { out << "]"; }
  }
  out << endl;
  return out;
}

}  // namespace jpcc::process
