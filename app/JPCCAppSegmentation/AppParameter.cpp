#include "AppParameter.h"

#include <filesystem>

namespace jpcc {

using namespace std;
using namespace po;

#define APP_OPT_PREFIX "app"
#define PARALLEL_OPT ".parallel"
#define HEADLESS_OPT ".headless"
#define GROUP_OF_FRAMES_SIZE_OPT ".groupOfFramesSize"

AppParameter::AppParameter() :
    Parameter(APP_OPT_PREFIX, __FUNCTION__),
    parallel(false),
    headless(true),
    inputDataset("dataset", "InputDatasetParameter"),
    inputReader("reader", "InputDatasetReaderParameter"),
    outputDataset("outputDataset", "OutputDatasetParameter"),
    jpccGmmSegmentation(),
    visualizerParameter() {
  opts_.add_options()                                                        //
      (string(prefix_ + PARALLEL_OPT).c_str(),                               //
       value<bool>(&parallel)->default_value(parallel),                      //
       "parallel")                                                           //
      (string(prefix_ + HEADLESS_OPT).c_str(),                               //
       value<bool>(&headless)->default_value(headless),                      //
       "headless")                                                           //
      (string(prefix_ + GROUP_OF_FRAMES_SIZE_OPT).c_str(),                   //
       value<size_t>(&groupOfFramesSize)->default_value(groupOfFramesSize),  //
       "groupOfFramesSize")                                                  //
      ;
  opts_.add(inputDataset.getOpts());
  opts_.add(inputReader.getOpts());
  opts_.add(outputDataset.getOpts());
  opts_.add(preProcess.getOpts());
  opts_.add(jpccNormalEstimation.getOpts());
  opts_.add(jpccGmmSegmentation.getOpts());
  opts_.add(visualizerParameter.getOpts());
}

void AppParameter::notify() {
  inputDataset.notify();
  inputReader.notify();
  outputDataset.notify(false);
  preProcess.notify();
  jpccNormalEstimation.notify();
  jpccGmmSegmentation.notify();
  visualizerParameter.notify();
  const filesystem::path& path = filesystem::path(outputDataset.getFilePath(0));
  if (!filesystem::exists(path.parent_path())) { filesystem::create_directories(path.parent_path()); }
}

ostream& operator<<(ostream& out, const AppParameter& obj) {
  obj.coutParameters(out)                                //
      (PARALLEL_OPT, obj.parallel)                       //
      (HEADLESS_OPT, obj.headless)                       //
      (GROUP_OF_FRAMES_SIZE_OPT, obj.groupOfFramesSize)  //
      ;
  out << obj.inputDataset;
  out << obj.inputReader;
  out << obj.outputDataset;
  out << obj.preProcess;
  out << obj.jpccNormalEstimation;
  out << obj.jpccGmmSegmentation;
  out << obj.visualizerParameter;
  return out;
}

}  // namespace jpcc