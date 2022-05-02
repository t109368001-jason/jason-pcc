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

void JPCCConditionalRemovalParameter::getShowTexts(vector<std::string>& showTexts) const {
  if (enable) {
    stringstream ss;
    ss << prefix_ << CONDITIONS_OPT ": ";
    for (size_t i = 0; i < conditions_.size(); i++) {
      if (i != 0) { ss << " && "; }
      ss << conditions_.at(i);
    }
    showTexts.push_back(ss.str());
  }
}

void JPCCConditionalRemovalParameter::notify() {
  if (enable) {
    assert(!conditions_.empty());
    conditions.resize(conditions_.size());
    std::transform(conditions_.begin(), conditions_.end(), conditions.begin(),
                   [](const string& condition) { return Condition(condition); });
  }
}

ostream& operator<<(ostream& out, const JPCCConditionalRemovalParameter& obj) {
  obj.coutParameters(out)                //
      (ENABLE_OPT, obj.enable)           //
      (CONDITIONS_OPT, obj.conditions_)  //
      ;
  return out;
}

}  // namespace jpcc::process
