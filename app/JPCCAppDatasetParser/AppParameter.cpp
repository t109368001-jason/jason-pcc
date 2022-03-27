#include "AppParameter.h"

#include <filesystem>

namespace jpcc {

using namespace std;
using namespace po;

#define APP_OPT_PREFIX "app"
#define PARALLEL_OPT_PREFIX ".parallel"

AppParameter::AppParameter() :
    Parameter(APP_OPT_PREFIX, __FUNCTION__),
    parallel(false),
    inputDataset("dataset", "InputDatasetParameter"),
    inputReader("reader", "InputDatasetReaderParameter"),
    outputDataset("outputDataset", "OutputDatasetParameter") {
  opts_.add_options()                                    //
      (string(prefix_ + PARALLEL_OPT_PREFIX).c_str(),    //
       value<bool>(&parallel)->default_value(parallel),  //
       "parallel")                                       //
      ;
  opts_.add(inputDataset.getOpts());
  opts_.add(inputReader.getOpts());
  opts_.add(outputDataset.getOpts());
}

void AppParameter::notify() {
  const filesystem::path& path = filesystem::path(outputDataset.getFilePath(0));
  if (!filesystem::exists(path.parent_path())) { filesystem::create_directories(path.parent_path()); }
  inputDataset.notify();
  inputReader.notify();
  outputDataset.notify();
}

ostream& operator<<(ostream& out, const AppParameter& obj) {
  out << obj.caption_ << endl;
  out << "\t" << obj.prefix_ << PARALLEL_OPT_PREFIX "=" << obj.parallel << endl;
  out << obj.inputDataset;
  out << obj.inputReader;
  out << obj.outputDataset;
  return out;
}

}  // namespace jpcc