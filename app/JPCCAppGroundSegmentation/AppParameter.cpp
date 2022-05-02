#include "AppParameter.h"

#include <boost/algorithm/string.hpp>

namespace jpcc {

using namespace std;
using namespace po;

#define APP_OPT_PREFIX "app"
#define PARALLEL_OPT_PREFIX ".parallel"
#define DISTANCE_THRESHOLD_OPT_PREFIX ".distanceThreshold"
#define HEADLESS_OPT_PREFIX ".headless"

AppParameter::AppParameter() :
    Parameter(APP_OPT_PREFIX, __FUNCTION__),
    parallel(false),
    distanceThreshold(0.5),
    headless(false),
    dataset(),
    reader(),
    preProcess() {
  opts_.add_options()                                                        //
      (string(prefix_ + PARALLEL_OPT_PREFIX).c_str(),                        //
       value<bool>(&parallel)->default_value(parallel),                      //
       "parallel")                                                           //
      (string(prefix_ + DISTANCE_THRESHOLD_OPT_PREFIX).c_str(),              //
       value<double>(&distanceThreshold)->default_value(distanceThreshold),  //
       "distanceThreshold")                                                  //
      (string(prefix_ + HEADLESS_OPT_PREFIX).c_str(),                        //
       value<bool>(&headless)->default_value(headless),                      //
       "headless")                                                           //
      ;
  opts_.add(dataset.getOpts());
  opts_.add(reader.getOpts());
  opts_.add(preProcess.getOpts());
  opts_.add(visualizerParameter.getOpts());
}

void AppParameter::getShowTexts(vector<std::string>& showTexts) const {
  showTexts.push_back(prefix_ + DISTANCE_THRESHOLD_OPT_PREFIX ": " + to_string(distanceThreshold));
  dataset.getShowTexts(showTexts);
  reader.getShowTexts(showTexts);
  preProcess.getShowTexts(showTexts);
  visualizerParameter.getShowTexts(showTexts);
}

void AppParameter::notify() {
  dataset.notify();
  reader.notify();
  preProcess.notify();
  visualizerParameter.notify();
}

ostream& operator<<(ostream& out, const AppParameter& obj) {
  obj.coutParameters(out)                                     //
      (PARALLEL_OPT_PREFIX, obj.parallel)                     //
      (DISTANCE_THRESHOLD_OPT_PREFIX, obj.distanceThreshold)  //
      (HEADLESS_OPT_PREFIX, obj.headless)                     //
      ;
  out << obj.dataset;
  out << obj.reader;
  out << obj.preProcess;
  out << obj.visualizerParameter;
  return out;
}

}  // namespace jpcc