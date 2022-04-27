#include "AppParameter.h"

#include <boost/algorithm/string.hpp>

namespace jpcc {

using namespace std;
using namespace po;

#define APP_OPT_PREFIX "app"
#define PARALLEL_OPT_PREFIX ".parallel"
#define DISTANCE_THRESHOLD_OPT_PREFIX ".distanceThreshold"

AppParameter::AppParameter() :
    Parameter(APP_OPT_PREFIX, __FUNCTION__),
    parallel(false),
    distanceThreshold(0.5),
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
      ;
  opts_.add(dataset.getOpts());
  opts_.add(reader.getOpts());
  opts_.add(preProcess.getOpts());
  opts_.add(visualizerParameter.getOpts());
}

void AppParameter::notify() {
  dataset.notify();
  reader.notify();
  preProcess.notify();
  visualizerParameter.notify();
}

ostream& operator<<(ostream& out, const AppParameter& obj) {
  out << obj.caption_ << endl;
  out << "\t" << obj.prefix_ << PARALLEL_OPT_PREFIX "=" << obj.parallel << endl;
  out << "\t" << obj.prefix_ << DISTANCE_THRESHOLD_OPT_PREFIX "=" << obj.distanceThreshold << endl;
  out << obj.dataset;
  out << obj.reader;
  out << obj.preProcess;
  out << obj.visualizerParameter;
  return out;
}

}  // namespace jpcc