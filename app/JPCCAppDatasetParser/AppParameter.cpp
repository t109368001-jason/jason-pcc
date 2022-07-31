#include "AppParameter.h"

#include <filesystem>

namespace jpcc {

using namespace std;
using namespace po;

#define APP_OPT_PREFIX "app"
#define PARALLEL_OPT ".parallel"
#define OUTPUT_POINT_TYPE_OPT ".outputPointType"

AppParameter::AppParameter() :
    Parameter(APP_OPT_PREFIX, __FUNCTION__),
    parallel(false),
    outputPointType("PointXYZ"),
    inputDataset("dataset", "InputDatasetParameter"),
    inputReader("reader", "InputDatasetReaderParameter"),
    outputDataset("outputDataset", "OutputDatasetParameter") {
  opts_.add_options()                                                    //
      (string(prefix_ + PARALLEL_OPT).c_str(),                           //
       value<bool>(&parallel)->default_value(parallel),                  //
       "parallel")                                                       //
      (string(prefix_ + OUTPUT_POINT_TYPE_OPT).c_str(),                  //
       value<string>(&outputPointType)->default_value(outputPointType),  //
       "outputPointType")                                                //
      ;
  opts_.add(inputDataset.getOpts());
  opts_.add(inputReader.getOpts());
  opts_.add(outputDataset.getOpts());
  opts_.add(preProcess.getOpts());
  opts_.add(jpccNormalEstimation.getOpts());
}

void AppParameter::notify() {
  inputDataset.notify();
  inputReader.notify();
  outputDataset.notify(false);
  preProcess.notify();
  jpccNormalEstimation.notify();
  const filesystem::path& path = filesystem::path(outputDataset.getFilePath(0));
  if (!filesystem::exists(path.parent_path())) { filesystem::create_directories(path.parent_path()); }
}

ostream& operator<<(ostream& out, const AppParameter& obj) {
  obj.coutParameters(out)                           //
      (PARALLEL_OPT, obj.parallel)                  //
      (OUTPUT_POINT_TYPE_OPT, obj.outputPointType)  //
      ;
  out << obj.inputDataset;
  out << obj.inputReader;
  out << obj.outputDataset;
  out << obj.preProcess;
  out << obj.jpccNormalEstimation;
  return out;
}

}  // namespace jpcc