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
                              "StatisticalOutlierRemovalParameter"),
    jpccConditionalRemovalParameter(prefix + "." + JPCC_CONDITIONAL_REMOVAL_OPT_PREFIX,
                                    "JPCCConditionalRemovalParameter") {
  opts_.add_options()                        //
      (string(prefix_ + ORDER_OPT).c_str(),  //
       value<string>(&order_),               //
       "order")                              //
      ;
  opts_.add(radiusOutlierRemoval.getOpts());
  opts_.add(statisticalOutlierRemoval.getOpts());
  opts_.add(jpccConditionalRemovalParameter.getOpts());
}

void PreProcessParameter::getShowTexts(vector<std::string>& showTexts) const {
  if (!order_.empty()) { showTexts.push_back(prefix_ + ORDER_OPT ": " + order_); }
  radiusOutlierRemoval.getShowTexts(showTexts);
  statisticalOutlierRemoval.getShowTexts(showTexts);
  jpccConditionalRemovalParameter.getShowTexts(showTexts);
}

void PreProcessParameter::notify() {
  radiusOutlierRemoval.notify();
  statisticalOutlierRemoval.notify();
  jpccConditionalRemovalParameter.notify();
  size_t algorithmCount = 0;
  if (radiusOutlierRemoval.enable) {
    assert(boost::icontains(order_, RADIUS_OUTLIER_REMOVAL_OPT_PREFIX));
    algorithmCount++;
  }
  if (statisticalOutlierRemoval.enable) {
    assert(boost::icontains(order_, STATISTICAL_OUTLIER_REMOVAL_OPT_PREFIX));
    algorithmCount++;
  }
  if (jpccConditionalRemovalParameter.enable) {
    assert(boost::icontains(order_, JPCC_CONDITIONAL_REMOVAL_OPT_PREFIX));
    algorithmCount++;
  }
  if (!order_.empty()) { boost::algorithm::split(order, order_, boost::is_any_of(",")); }
  assert(order.size() == algorithmCount);
}

ostream& operator<<(ostream& out, const PreProcessParameter& obj) {
  out << obj.caption_ << endl;
  out << "\t" << obj.prefix_ << ORDER_OPT "=" << obj.order_ << endl;
  out << obj.radiusOutlierRemoval;
  out << obj.statisticalOutlierRemoval;
  out << obj.jpccConditionalRemovalParameter;
  return out;
}

}  // namespace jpcc::process
