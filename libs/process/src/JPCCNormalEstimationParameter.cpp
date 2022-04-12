#include <jpcc/process/JPCCNormalEstimationParameter.h>

namespace jpcc::process {

using namespace std;
using namespace po;

#define K_SEARCH_OPT ".kSearch"
#define RADIUS_SEARCH_OPT ".radiusSearch"

JPCCNormalEstimationParameter::JPCCNormalEstimationParameter() :
    JPCCNormalEstimationParameter(JPCC_NORMAL_ESTIMATION_OPT_PREFIX, __FUNCTION__) {}

JPCCNormalEstimationParameter::JPCCNormalEstimationParameter(const std::string& prefix, const std::string& caption) :
    Parameter(prefix, caption), kSearch(0), radiusSearch(0) {
  opts_.add_options()                                           //
      (string(prefix_ + K_SEARCH_OPT).c_str(),                  //
       value<int>(&kSearch)->default_value(kSearch),            //
       JPCC_NORMAL_ESTIMATION_OPT_PREFIX " kSearch")            //
      (string(prefix_ + RADIUS_SEARCH_OPT).c_str(),             //
       value<int>(&radiusSearch)->default_value(radiusSearch),  //
       JPCC_NORMAL_ESTIMATION_OPT_PREFIX " radiusSearch")       //
      ;
}

void JPCCNormalEstimationParameter::notify() {
  assert((radiusSearch > 0 && kSearch == 0) || (radiusSearch == 0 && kSearch > 0));
}

ostream& operator<<(ostream& out, const JPCCNormalEstimationParameter& obj) {
  out << obj.caption_ << endl;
  out << "\t" << obj.prefix_ << K_SEARCH_OPT "=" << obj.kSearch << endl;
  out << "\t" << obj.prefix_ << RADIUS_SEARCH_OPT "=" << obj.radiusSearch << endl;
  return out;
}

}  // namespace jpcc::process
