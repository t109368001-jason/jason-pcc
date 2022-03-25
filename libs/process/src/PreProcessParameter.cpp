#include <jpcc/process/PreProcessParameter.h>

#include <boost/algorithm/string.hpp>

namespace jpcc::process {

using namespace std;
using namespace po;

#define PRE_PROCESS_OPT_PREFIX "preProcess"
#define ORDER_OPT ".order"

PreProcessParameter::PreProcessParameter() : PreProcessParameter(PRE_PROCESS_OPT_PREFIX, __FUNCTION__) {}

PreProcessParameter::PreProcessParameter(const std::string& prefix, const std::string& caption) :
    Parameter(prefix, caption),
    radiusOutlierRemoval(prefix + "." + RADIUS_OUTLIER_REMOVAL_OPT_PREFIX, "RadiusOutlierRemovalParameter") {
  opts_.add_options()                        //
      (string(prefix_ + ORDER_OPT).c_str(),  //
       value<string>(&order_),               //
       "order")                              //
      ;
  opts_.add(radiusOutlierRemoval.getOpts());
}

ostream& operator<<(ostream& out, const PreProcessParameter& obj) {
  out << obj.caption_ << endl;
  out << "\t" << obj.prefix_ << ORDER_OPT "=" << obj.order_ << endl;
  out << obj.radiusOutlierRemoval;
  return out;
}

void PreProcessParameter::notify() {
  radiusOutlierRemoval.notify();
  if (radiusOutlierRemoval.enable) { assert(boost::icontains(order_, RADIUS_OUTLIER_REMOVAL_OPT_PREFIX)); }
  boost::algorithm::split(order, order_, boost::is_any_of(","));
}

}  // namespace jpcc::process
