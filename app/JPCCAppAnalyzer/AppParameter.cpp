#include "AppParameter.h"

#include <boost/algorithm/string.hpp>

namespace jpcc {

using namespace std;
using namespace po;

#define APP_OPT_PREFIX "app"
#define PARALLEL_OPT_PREFIX ".parallel"
#define RESOLUTION_OPT_PREFIX ".resolution"
#define OUTPUT_CSV_PATH_OPT_PREFIX ".outputCSVPath"

AppParameter::AppParameter() :
    Parameter(APP_OPT_PREFIX, __FUNCTION__),
    parallel(false),
    resolution(0.1),
    outputCSVPath(),
    dataset(),
    reader(),
    preProcess() {
  opts_.add_options()                                                //
      (string(prefix_ + PARALLEL_OPT_PREFIX).c_str(),                //
       value<bool>(&parallel)->default_value(parallel),              //
       "parallel")                                                   //
      (string(prefix_ + RESOLUTION_OPT_PREFIX).c_str(),              //
       value<double>(&resolution)->default_value(resolution),        //
       "resolution")                                                 //
      (string(prefix_ + OUTPUT_CSV_PATH_OPT_PREFIX).c_str(),         //
       value<string>(&outputCSVPath)->default_value(outputCSVPath),  //
       "outputCSVPath")                                              //
      ;
  opts_.add(dataset.getOpts());
  opts_.add(reader.getOpts());
  opts_.add(preProcess.getOpts());
}

void AppParameter::getShowTexts(vector<std::string>& showTexts) const {
  showTexts.push_back(prefix_ + RESOLUTION_OPT_PREFIX ": " + to_string(resolution));
  dataset.getShowTexts(showTexts);
  reader.getShowTexts(showTexts);
  preProcess.getShowTexts(showTexts);
}

void AppParameter::notify() {
  dataset.notify();
  reader.notify();
  preProcess.notify();
}

ostream& operator<<(ostream& out, const AppParameter& obj) {
  obj.coutParameters(out)                              //
      (PARALLEL_OPT_PREFIX, obj.parallel)              //
      (RESOLUTION_OPT_PREFIX, obj.resolution)          //
      (OUTPUT_CSV_PATH_OPT_PREFIX, obj.outputCSVPath)  //
      ;
  out << obj.dataset;
  out << obj.reader;
  out << obj.preProcess;
  return out;
}

}  // namespace jpcc