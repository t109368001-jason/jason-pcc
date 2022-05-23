#include "AppParameter.h"

#include <boost/algorithm/string.hpp>

namespace jpcc {

using namespace std;
using namespace po;

#define APP_OPT_PREFIX "app"
#define PARALLEL_OPT_PREFIX ".parallel"
#define PREVIEW_ONLY_OPT_PREFIX ".previewOnly"
#define RESOLUTION_OPT_PREFIX ".resolution"
#define OUTPUT_CSV_PATH_OPT_PREFIX ".outputCSVPath"

AppParameter::AppParameter() :
    Parameter(APP_OPT_PREFIX, __FUNCTION__),
    parallel(false),
    previewOnly(false),
    resolution(0.1),
    outputCSVPath(),
    dataset(),
    reader(),
    preProcess(),
    background(string("background.") + JPCC_CONDITIONAL_REMOVAL_OPT_PREFIX, "JPCCConditionalRemovalParameter"),
    dynamic(string("dynamic.") + JPCC_CONDITIONAL_REMOVAL_OPT_PREFIX, "JPCCConditionalRemovalParameter"),
    visualizerParameter() {
  opts_.add_options()                                                //
      (string(prefix_ + PARALLEL_OPT_PREFIX).c_str(),                //
       value<bool>(&parallel)->default_value(parallel),              //
       "parallel")                                                   //
      (string(prefix_ + PREVIEW_ONLY_OPT_PREFIX).c_str(),            //
       value<bool>(&previewOnly)->default_value(previewOnly),        //
       "previewOnly")                                                //
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
  opts_.add(background.getOpts());
  opts_.add(dynamic.getOpts());
  opts_.add(jpccNormalEstimation.getOpts());
  opts_.add(visualizerParameter.getOpts());
}

void AppParameter::getShowTexts(vector<std::string>& showTexts) const {
  showTexts.push_back(prefix_ + RESOLUTION_OPT_PREFIX ": " + to_string(resolution));
  dataset.getShowTexts(showTexts);
  reader.getShowTexts(showTexts);
  preProcess.getShowTexts(showTexts);
  background.getShowTexts(showTexts);
  dynamic.getShowTexts(showTexts);
  jpccNormalEstimation.getShowTexts(showTexts);
  visualizerParameter.getShowTexts(showTexts);
}

void AppParameter::notify() {
  dataset.notify();
  reader.notify();
  preProcess.notify();
  background.notify();
  dynamic.notify();
  jpccNormalEstimation.notify();
  visualizerParameter.notify();
}

ostream& operator<<(ostream& out, const AppParameter& obj) {
  obj.coutParameters(out)                              //
      (PARALLEL_OPT_PREFIX, obj.parallel)              //
      (PREVIEW_ONLY_OPT_PREFIX, obj.previewOnly)       //
      (RESOLUTION_OPT_PREFIX, obj.resolution)          //
      (OUTPUT_CSV_PATH_OPT_PREFIX, obj.outputCSVPath)  //
      ;
  out << obj.dataset;
  out << obj.reader;
  out << obj.preProcess;
  out << obj.background;
  out << obj.dynamic;
  out << obj.jpccNormalEstimation;
  out << obj.visualizerParameter;
  return out;
}

}  // namespace jpcc