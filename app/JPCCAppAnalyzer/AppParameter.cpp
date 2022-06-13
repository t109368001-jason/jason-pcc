#include "AppParameter.h"

#include <filesystem>

#include <boost/algorithm/string.hpp>

namespace jpcc {

using namespace std;
using namespace std::filesystem;
using namespace po;

#define APP_OPT_PREFIX "app"
#define PARALLEL_OPT ".parallel"
#define ANALYZE_PARALLEL_OPT ".analyzeParallel"
#define PREVIEW_ONLY_OPT ".previewOnly"
#define FORCE_RE_RUN_OPT ".forceReRun"
#define RESOLUTION_OPT ".resolution"
#define OUTPUT_DIR_OPT ".outputDir"

AppParameter::AppParameter() :
    Parameter(APP_OPT_PREFIX, __FUNCTION__),
    parallel(false),
    analyzeParallel(false),
    previewOnly(false),
    forceReRun(false),
    resolution(0.1),
    dataset(),
    reader(),
    background(string("background.") + JPCC_CONDITIONAL_REMOVAL_OPT_PREFIX, "JPCCConditionalRemovalParameter"),
    dynamic(string("dynamic.") + JPCC_CONDITIONAL_REMOVAL_OPT_PREFIX, "JPCCConditionalRemovalParameter"),
    visualizerParameter() {
  opts_.add_options()                                                  //
      (string(prefix_ + PARALLEL_OPT).c_str(),                         //
       value<bool>(&parallel)->default_value(parallel),                //
       "parallel")                                                     //
      (string(prefix_ + ANALYZE_PARALLEL_OPT).c_str(),                 //
       value<bool>(&analyzeParallel)->default_value(analyzeParallel),  //
       "analyzeParallel")                                              //
      (string(prefix_ + PREVIEW_ONLY_OPT).c_str(),                     //
       value<bool>(&previewOnly)->default_value(previewOnly),          //
       "previewOnly")                                                  //
      (string(prefix_ + FORCE_RE_RUN_OPT).c_str(),                     //
       value<bool>(&forceReRun)->default_value(forceReRun),            //
       "forceReRun")                                                   //
      (string(prefix_ + RESOLUTION_OPT).c_str(),                       //
       value<double>(&resolution)->default_value(resolution),          //
       "resolution")                                                   //
      (string(prefix_ + OUTPUT_DIR_OPT).c_str(),                       //
       value<string>(&outputDir)->default_value(outputDir),            //
       "outputDir")                                                    //
      ;
  opts_.add(dataset.getOpts());
  opts_.add(reader.getOpts());
  opts_.add(background.getOpts());
  opts_.add(dynamic.getOpts());
  opts_.add(visualizerParameter.getOpts());
}

void AppParameter::getShowTexts(vector<std::string>& showTexts) const {
  showTexts.push_back(prefix_ + RESOLUTION_OPT ": " + to_string(resolution));
  dataset.getShowTexts(showTexts);
  reader.getShowTexts(showTexts);
  background.getShowTexts(showTexts);
  dynamic.getShowTexts(showTexts);
  visualizerParameter.getShowTexts(showTexts);
}

void AppParameter::notify() {
  dataset.notify();
  if (!dataset.preProcessed) {
    BOOST_THROW_EXCEPTION(std::logic_error(std::string("only support pre-processed dataset")));
  }
  reader.notify();
  background.notify();
  dynamic.notify();
  visualizerParameter.notify();
  if (!exists(outputDir)) { create_directory(outputDir); }
  assert(exists(outputDir) && is_directory(outputDir));
}

ostream& operator<<(ostream& out, const AppParameter& obj) {
  obj.coutParameters(out)                          //
      (PARALLEL_OPT, obj.parallel)                 //
      (ANALYZE_PARALLEL_OPT, obj.analyzeParallel)  //
      (PREVIEW_ONLY_OPT, obj.previewOnly)          //
      (FORCE_RE_RUN_OPT, obj.forceReRun)           //
      (RESOLUTION_OPT, obj.resolution)             //
      ;
  out << obj.dataset;
  out << obj.reader;
  out << obj.background;
  out << obj.dynamic;
  out << obj.visualizerParameter;
  return out;
}

}  // namespace jpcc