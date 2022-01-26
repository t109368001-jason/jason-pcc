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
    inputDatasetParameter("dataset", "InputDatasetParameter"),
    inputDatasetReaderParameter("reader", "InputDatasetReaderParameter"),
    outputDatasetParameter("outputDataset", "OutputDatasetParameter") {
  opts_.add_options()                                    //
      (string(prefix_ + PARALLEL_OPT_PREFIX).c_str(),    //
       value<bool>(&parallel)->default_value(parallel),  //
       "parallel")                                       //
      ;
}

void AppParameter::notify() {
  const filesystem::path& path = filesystem::path(outputDatasetParameter.getFilePath(0));
  if (!filesystem::exists(path.parent_path())) { filesystem::create_directories(path.parent_path()); }
}

}  // namespace jpcc