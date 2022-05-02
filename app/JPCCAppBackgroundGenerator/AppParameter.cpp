#include "AppParameter.h"

#include <boost/algorithm/string.hpp>

namespace jpcc {

using namespace std;
using namespace po;

#define APP_OPT_PREFIX "app"
#define PARALLEL_OPT ".parallel"
#define GROUP_OF_FRAMES_SIZE_OPT ".groupOfFramesSize"
#define OUTPUT_PATH_OPT ".outputPath"
#define BACKGROUND_THRESHOLD_OPT ".backgroundThreshold"
#define FILTER_RESOLUTION_OPT ".filterResolution"
#define BACKGROUND_RESOLUTION_OPT ".backgroundResolution"

AppParameter::AppParameter() :
    Parameter(APP_OPT_PREFIX, __FUNCTION__),
    parallel(false),
    groupOfFramesSize(32),
    outputPath(),
    dataset(),
    reader(),
    preProcess(),
    jpccNormalEstimation() {
  opts_.add_options()                                                              //
      (string(prefix_ + PARALLEL_OPT).c_str(),                                     //
       value<bool>(&parallel)->default_value(parallel),                            //
       "parallel")                                                                 //
      (string(prefix_ + GROUP_OF_FRAMES_SIZE_OPT).c_str(),                         //
       value<size_t>(&groupOfFramesSize)->default_value(groupOfFramesSize),        //
       "groupOfFramesSize")                                                        //
      (string(prefix_ + OUTPUT_PATH_OPT).c_str(),                                  //
       value<string>(&outputPath)->default_value(outputPath),                      //
       "outputPath")                                                               //
      (string(prefix_ + BACKGROUND_THRESHOLD_OPT).c_str(),                         //
       value<double>(&backgroundThreshold)->default_value(backgroundThreshold),    //
       "backgroundThreshold")                                                      //
      (string(prefix_ + FILTER_RESOLUTION_OPT).c_str(),                            //
       value<double>(&filterResolution)->default_value(filterResolution),          //
       "filterResolution")                                                         //
      (string(prefix_ + BACKGROUND_RESOLUTION_OPT).c_str(),                        //
       value<double>(&backgroundResolution)->default_value(backgroundResolution),  //
       "backgroundResolution")                                                     //
      ;
  opts_.add(dataset.getOpts());
  opts_.add(reader.getOpts());
  opts_.add(preProcess.getOpts());
  opts_.add(jpccNormalEstimation.getOpts());
}

void AppParameter::notify() {
  dataset.notify();
  reader.notify();
  preProcess.notify();
  jpccNormalEstimation.notify();
}

std::string AppParameter::getOutputPath() const {
  if (!outputPath.empty()) {
    return outputPath;
  } else {
    return (dataset.folderPath / "background.ply").string();
  }
}

ostream& operator<<(ostream& out, const AppParameter& obj) {
  obj.coutParameters(out)                                    //
      (PARALLEL_OPT, obj.parallel)                           //
      (GROUP_OF_FRAMES_SIZE_OPT, obj.groupOfFramesSize)      //
      (OUTPUT_PATH_OPT, obj.outputPath)                      //
      (FILTER_RESOLUTION_OPT, obj.filterResolution)          //
      (BACKGROUND_RESOLUTION_OPT, obj.backgroundResolution)  //
      ;
  out << obj.dataset;
  out << obj.reader;
  out << obj.preProcess;
  out << obj.jpccNormalEstimation;
  return out;
}

}  // namespace jpcc