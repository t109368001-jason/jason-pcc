#include <jpcc/process/StatisticalOutlierRemovalParameter.h>

namespace jpcc::process {

using namespace std;
using namespace po;

#define ENABLE_OPT ".enable"
#define MEAN_K_OPT ".meanK"
#define STDDEV_MUL_THRESH_OPT ".stddevMulThresh"

StatisticalOutlierRemovalParameter::StatisticalOutlierRemovalParameter() :
    StatisticalOutlierRemovalParameter(STATISTICAL_OUTLIER_REMOVAL_OPT_PREFIX, __FUNCTION__) {}

StatisticalOutlierRemovalParameter::StatisticalOutlierRemovalParameter(const std::string& prefix,
                                                                       const std::string& caption) :
    Parameter(prefix, caption), enable(false), meanK(50), stddevMulThresh(1.0) {
  opts_.add_options()                                    //
      (string(prefix_ + ENABLE_OPT).c_str(),             //
       value<bool>(&enable),                             //
       "statisticalOutlierRemoval enable")               //
      (string(prefix_ + MEAN_K_OPT).c_str(),             //
       value<size_t>(&meanK),                            //
       "statisticalOutlierRemoval meanK")                //
      (string(prefix_ + STDDEV_MUL_THRESH_OPT).c_str(),  //
       value<float>(&stddevMulThresh),                   //
       "statisticalOutlierRemoval stddevMulThresh")      //
      ;
}

void StatisticalOutlierRemovalParameter::notify() {
  if (enable) {
    assert(meanK > 0);
    assert(stddevMulThresh > 0.0);
  }
}

ostream& operator<<(ostream& out, const StatisticalOutlierRemovalParameter& obj) {
  out << obj.caption_ << endl;
  out << "\t" << obj.prefix_ << ENABLE_OPT "=" << obj.enable << endl;
  out << "\t" << obj.prefix_ << MEAN_K_OPT "=" << obj.meanK << endl;
  out << "\t" << obj.prefix_ << STDDEV_MUL_THRESH_OPT "=" << obj.stddevMulThresh << endl;
  return out;
}

}  // namespace jpcc::process
