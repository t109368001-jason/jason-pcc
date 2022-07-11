#include "AppParameter.h"

#include <boost/algorithm/string.hpp>

namespace jpcc {

using namespace std;
using namespace po;

#define APP_OPT_PREFIX "app"
#define PARALLEL_OPT ".parallel"
#define HEADLESS_OPT ".headless"
#define REGISTRATION_OPT ".registration"
#define RESOLUTION_OPT ".resolution"

AppParameter::AppParameter() :
    Parameter(APP_OPT_PREFIX, __FUNCTION__), parallel(false), registration("none"), dataset(), reader(), preProcess() {
  opts_.add_options()                                              //
      (string(prefix_ + PARALLEL_OPT).c_str(),                     //
       value<bool>(&parallel)->default_value(parallel),            //
       "parallel")                                                 //
      (string(prefix_ + HEADLESS_OPT).c_str(),                     //
       value<bool>(&parallel)->default_value(headless),            //
       "headless")                                                 //
      (string(prefix_ + REGISTRATION_OPT).c_str(),                 //
       value<string>(&registration)->default_value(registration),  //
       "registration")                                             //
      (string(prefix_ + RESOLUTION_OPT).c_str(),                   //
       value<float>(&resolution)->default_value(resolution),       //
       "resolution")                                               //
      ;
  opts_.add(dataset.getOpts());
  opts_.add(reader.getOpts());
  opts_.add(preProcess.getOpts());
  opts_.add(visualizerParameter.getOpts());
}

void AppParameter::getShowTexts(vector<std::string>& showTexts) const {
  showTexts.push_back(prefix_ + REGISTRATION_OPT ": " + registration);
  showTexts.push_back(prefix_ + RESOLUTION_OPT ": " + to_string(resolution));
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
  obj.coutParameters(out)                   //
      (PARALLEL_OPT, obj.parallel)          //
      (HEADLESS_OPT, obj.headless)          //
      (REGISTRATION_OPT, obj.registration)  //
      (RESOLUTION_OPT, obj.resolution)      //
      ;
  out << obj.dataset;
  out << obj.reader;
  out << obj.preProcess;
  out << obj.visualizerParameter;
  return out;
}

}  // namespace jpcc