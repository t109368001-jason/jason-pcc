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
    radiusOutlierRemoval(prefix + "." + RADIUS_OUTLIER_REMOVAL_OPT_PREFIX, "RadiusOutlierRemovalParameter"),
    statisticalOutlierRemoval(prefix + "." + STATISTICAL_OUTLIER_REMOVAL_OPT_PREFIX,
                              "StatisticalOutlierRemovalParameter") {
  opts_.add_options()                        //
      (string(prefix_ + ORDER_OPT).c_str(),  //
       value<string>(&order_),               //
       "order")                              //
      ;
  opts_.add(radiusOutlierRemoval.getOpts());
  opts_.add(statisticalOutlierRemoval.getOpts());
}

ostream& operator<<(ostream& out, const PreProcessParameter& obj) {
  out << obj.caption_ << endl;
  out << "\t" << obj.prefix_ << ORDER_OPT "=" << obj.order_ << endl;
  out << obj.radiusOutlierRemoval;
  out << obj.statisticalOutlierRemoval;
  return out;
}

void PreProcessParameter::notify() {
  radiusOutlierRemoval.notify();
  statisticalOutlierRemoval.notify();
  size_t algorithmCount = 0;
  if (radiusOutlierRemoval.enable) {
    assert(boost::icontains(order_, RADIUS_OUTLIER_REMOVAL_OPT_PREFIX));
    algorithmCount++;
  }
  if (statisticalOutlierRemoval.enable) {
    assert(boost::icontains(order_, STATISTICAL_OUTLIER_REMOVAL_OPT_PREFIX));
    algorithmCount++;
  }
  boost::algorithm::split(order, order_, boost::is_any_of(","));
  assert(order.size() == algorithmCount);
}

}  // namespace jpcc::process
