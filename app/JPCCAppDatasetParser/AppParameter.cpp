#include "AppParameter.h"

#include <filesystem>

namespace jpcc {

using namespace std;
using namespace po;

#define APP_OPT_PREFIX "app"
#define PARALLEL_OPT ".parallel"

AppParameter::AppParameter() :
    Parameter(APP_OPT_PREFIX, __FUNCTION__),
    parallel(false),
    inputDataset("dataset", "InputDatasetParameter"),
    inputReader("reader", "InputDatasetReaderParameter"),
    outputDataset("outputDataset", "OutputDatasetParameter") {
  opts_.add_options()                                    //
      (string(prefix_ + PARALLEL_OPT).c_str(),           //
       value<bool>(&parallel)->default_value(parallel),  //
       "parallel")                                       //
      ;
  opts_.add(inputDataset.getOpts());
  opts_.add(inputReader.getOpts());
  opts_.add(outputDataset.getOpts());
  opts_.add(preProcess.getOpts());
}

void AppParameter::notify() {
  inputDataset.notify();
  inputReader.notify();
  outputDataset.notify(false);
  preProcess.notify();
  const filesystem::path& path = filesystem::path(outputDataset.getFilePath(0));
  if (!filesystem::exists(path.parent_path())) { filesystem::create_directories(path.parent_path()); }
}

ostream& operator<<(ostream& out, const AppParameter& obj) {
  obj.coutParameters(out)           //
      (PARALLEL_OPT, obj.parallel)  //
      ;
  out << obj.inputDataset;
  out << obj.inputReader;
  out << obj.outputDataset;
  out << obj.preProcess;
  return out;
}

}  // namespace jpcc