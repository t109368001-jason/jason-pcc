#include <jpcc/process/JPCCConditionalRemovalParameter.h>

#include <algorithm>

#include <boost/algorithm/string.hpp>

namespace jpcc::process {

using namespace std;
using namespace po;

#define ENABLE_OPT ".enable"
#define TYPE_OPT ".type"
#define CONDITIONS_OPT ".conditions"

JPCCConditionalRemovalParameter::JPCCConditionalRemovalParameter() :
    JPCCConditionalRemovalParameter(JPCC_CONDITIONAL_REMOVAL_OPT_PREFIX, __FUNCTION__) {
}

JPCCConditionalRemovalParameter::JPCCConditionalRemovalParameter(const string& prefix, const string& caption) :
    Parameter(prefix, caption), conditions_(), type_("and"), enable(false), type(Condition::AND), condition() {
  opts_.add_options()                                //
      (string(prefix_ + ENABLE_OPT).c_str(),         //
       value<bool>(&enable),                         //
       "jpccConditionalRemoval enable")              //
      (string(prefix_ + TYPE_OPT).c_str(),           //
       value<string>(&type_)->default_value(type_),  //
       "jpccConditionalRemoval type_")               //
      (string(prefix_ + CONDITIONS_OPT).c_str(),     //
       value<vector<string>>(&conditions_),          //
       "jpccConditionalRemoval conditions")          //
      ;
}

void JPCCConditionalRemovalParameter::getShowTexts(vector<string>& showTexts) const {
  if (enable) {
    stringstream ss;
    ss << prefix_ << CONDITIONS_OPT ": ";
    for (size_t i = 0; i < conditions_.size(); i++) {
      if (i != 0) {
        ss << (type == Condition::AND ? " && " : " || ");
      }
      ss << conditions_[i];
    }
    showTexts.push_back(ss.str());
  }
}

void JPCCConditionalRemovalParameter::notify() {
  if (enable) {
    const string o = boost::to_lower_copy(type_);
    if (o == "and" || o == "&&") {
      type = Condition::AND;
    } else if (o == "or" || o == "||") {
      type = Condition::OR;
    } else {
      BOOST_THROW_EXCEPTION(logic_error("not support type: " + type_));
    }
    assert(!conditions_.empty());
    condition = Condition(type, conditions_);
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
