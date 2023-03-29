#include "AppParameter.h"

#include <filesystem>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

namespace jpcc {

using namespace std;
using namespace std::filesystem;
using namespace po;

#define APP_OPT_PREFIX "app"
#define PARALLEL_OPT ".parallel"
#define PREVIEW_ONLY_OPT ".previewOnly"
#define FORCE_RE_RUN_OPT ".forceReRun"
#define FREQUENCIES_OPT ".frequencies"
#define RESOLUTIONS_OPT ".resolutions"
#define QUANT_RESOLUTIONS_OPT ".quantResolutions"
#define OUTPUT_DIR_OPT ".outputDir"

AppParameter::AppParameter() :
    Parameter(APP_OPT_PREFIX, __FUNCTION__),
    parallel(false),
    previewOnly(false),
    forceReRun(false),
    frequencies(),
    resolutions(100),
    quantResolutions(5),
    dataset(),
    reader(),
    background(string("background.") + JPCC_CONDITIONAL_REMOVAL_OPT_PREFIX, "JPCCConditionalRemovalParameter"),
    dynamic(string("dynamic.") + JPCC_CONDITIONAL_REMOVAL_OPT_PREFIX, "JPCCConditionalRemovalParameter"),
    preProcess(),
    normalEstimation(),
    visualizerParameter() {
  opts_.add_options()                                          //
      (string(prefix_ + PARALLEL_OPT).c_str(),                 //
       value<bool>(&parallel)->default_value(parallel),        //
       "parallel")                                             //
      (string(prefix_ + PREVIEW_ONLY_OPT).c_str(),             //
       value<bool>(&previewOnly)->default_value(previewOnly),  //
       "previewOnly")                                          //
      (string(prefix_ + FORCE_RE_RUN_OPT).c_str(),             //
       value<bool>(&forceReRun)->default_value(forceReRun),    //
       "forceReRun")                                           //
      (string(prefix_ + FREQUENCIES_OPT).c_str(),              //
       value<vector<float>>(&frequencies),                     //
       "frequencies")                                          //
      (string(prefix_ + RESOLUTIONS_OPT).c_str(),              //
       value<vector<double>>(&resolutions),                    //
       "resolutions")                                          //
      (string(prefix_ + QUANT_RESOLUTIONS_OPT).c_str(),        //
       value<vector<size_t>>(&quantResolutions),               //
       "quantResolutions")                                     //
      (string(prefix_ + OUTPUT_DIR_OPT).c_str(),               //
       value<string>(&outputDir)->default_value(outputDir),    //
       "outputDir")                                            //
      ;
  opts_.add(dataset.getOpts());
  opts_.add(reader.getOpts());
  opts_.add(background.getOpts());
  opts_.add(dynamic.getOpts());
  opts_.add(preProcess.getOpts());
  opts_.add(normalEstimation.getOpts());
  opts_.add(visualizerParameter.getOpts());
}

void AppParameter::getShowTexts(vector<string>& showTexts) const {
  dataset.getShowTexts(showTexts);
  reader.getShowTexts(showTexts);
  background.getShowTexts(showTexts);
  dynamic.getShowTexts(showTexts);
  preProcess.getShowTexts(showTexts);
  normalEstimation.getShowTexts(showTexts);
  visualizerParameter.getShowTexts(showTexts);
}

void AppParameter::notify() {
  dataset.notify();
  reader.notify();
  background.notify();
  dynamic.notify();
  preProcess.notify();
  normalEstimation.notify();
  visualizerParameter.notify();
  if (!boost::iends_with(outputDir, "/")) {
    outputDir += "/";
  }
}

ostream& operator<<(ostream& out, const AppParameter& obj) {
  obj.coutParameters(out)                            //
      (PARALLEL_OPT, obj.parallel)                   //
      (PREVIEW_ONLY_OPT, obj.previewOnly)            //
      (FORCE_RE_RUN_OPT, obj.forceReRun)             //
      (FREQUENCIES_OPT, obj.frequencies)             //
      (RESOLUTIONS_OPT, obj.resolutions)             //
      (QUANT_RESOLUTIONS_OPT, obj.quantResolutions)  //
      (OUTPUT_DIR_OPT, obj.outputDir)                //
      ;
  out << obj.dataset;
  out << obj.reader;
  out << obj.background;
  out << obj.dynamic;
  out << obj.preProcess;
  out << obj.normalEstimation;
  out << obj.visualizerParameter;
  return out;
}

}  // namespace jpcc