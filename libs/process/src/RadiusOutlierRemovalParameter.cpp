#include <jpcc/process/RadiusOutlierRemovalParameter.h>

namespace jpcc::process {

using namespace std;
using namespace po;

#define ENABLE_OPT ".enable"
#define RADIUS_OPT ".radius"
#define MIN_NEIGHBORS_IN_RADIUS_OPT ".minNeighborsInRadius"

RadiusOutlierRemovalParameter::RadiusOutlierRemovalParameter() :
    RadiusOutlierRemovalParameter(RADIUS_OUTLIER_REMOVAL_OPT_PREFIX, __FUNCTION__) {}

RadiusOutlierRemovalParameter::RadiusOutlierRemovalParameter(const string& prefix, const string& caption) :
    Parameter(prefix, caption), enable(false), radius(1.0), minNeighborsInRadius(1) {
  opts_.add_options()                                          //
      (string(prefix_ + ENABLE_OPT).c_str(),                   //
       value<bool>(&enable),                                   //
       "radiusOutlierRemoval enable")                          //
      (string(prefix_ + RADIUS_OPT).c_str(),                   //
       value<float>(&radius),                                  //
       "radiusOutlierRemoval radius")                          //
      (string(prefix_ + MIN_NEIGHBORS_IN_RADIUS_OPT).c_str(),  //
       value<size_t>(&minNeighborsInRadius),                   //
       "radiusOutlierRemoval minNeighborsInRadius")            //
      ;
}

void RadiusOutlierRemovalParameter::getShowTexts(vector<string>& showTexts) const {
  if (enable) {
    showTexts.push_back(prefix_ + RADIUS_OPT ": " + to_string(radius));
    showTexts.push_back(prefix_ + MIN_NEIGHBORS_IN_RADIUS_OPT ": " + to_string(minNeighborsInRadius));
  }
}

void RadiusOutlierRemovalParameter::notify() {
  if (enable) {
    THROW_IF_NOT(radius > 0.0);
    THROW_IF_NOT(minNeighborsInRadius > 0);
  }
}

ostream& operator<<(ostream& out, const RadiusOutlierRemovalParameter& obj) {
  obj.coutParameters(out)                                      //
      (ENABLE_OPT, obj.enable)                                 //
      (RADIUS_OPT, obj.radius)                                 //
      (MIN_NEIGHBORS_IN_RADIUS_OPT, obj.minNeighborsInRadius)  //
      ;
  return out;
}

}  // namespace jpcc::process
