#include <jpcc/process/RadiusOutlierRemovalParameter.h>

namespace jpcc::process {

using namespace std;
using namespace po;

#define ENABLE_OPT ".enable"
#define RADIUS_OPT ".radius"
#define MIN_NEIGHBORS_IN_RADIUS_OPT ".minNeighborsInRadius"

RadiusOutlierRemovalParameter::RadiusOutlierRemovalParameter() :
    RadiusOutlierRemovalParameter(RADIUS_OUTLIER_REMOVAL_OPT_PREFIX, __FUNCTION__) {}

RadiusOutlierRemovalParameter::RadiusOutlierRemovalParameter(const std::string& prefix, const std::string& caption) :
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

void RadiusOutlierRemovalParameter::notify() {
  if (enable) {
    assert(radius > 0.0);
    assert(minNeighborsInRadius > 0);
  }
}

ostream& operator<<(ostream& out, const RadiusOutlierRemovalParameter& obj) {
  out << obj.caption_ << endl;
  out << "\t" << obj.prefix_ << ENABLE_OPT "=" << obj.enable << endl;
  out << "\t" << obj.prefix_ << RADIUS_OPT "=" << obj.radius << endl;
  out << "\t" << obj.prefix_ << MIN_NEIGHBORS_IN_RADIUS_OPT "=" << obj.minNeighborsInRadius << endl;
  return out;
}

}  // namespace jpcc::process
