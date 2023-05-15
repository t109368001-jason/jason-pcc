#include "AppParameter.h"

#include <filesystem>

namespace jpcc {

using namespace std;
using namespace po;

#define APP_OPT_PREFIX "app"
#define PARALLEL_OPT ".parallel"
#define GROUP_OF_FRAMES_SIZE_OPT ".groupOfFramesSize"
#define OFFSET_OPT ".offset"
#define QP_OPT ".qp"

AppParameter::AppParameter() :
    Parameter(APP_OPT_PREFIX, __FUNCTION__),
    parallel(false),
    groupOfFramesSize(32),
    qp(1),
    inputDataset("dataset", "InputDatasetParameter"),
    inputReader("reader", "InputDatasetReaderParameter"),
    outputDataset("outputDataset", "OutputDatasetParameter") {
  opts_.add_options()                                                        //
      (string(prefix_ + PARALLEL_OPT).c_str(),                               //
       value<bool>(&parallel)->default_value(parallel),                      //
       "parallel")                                                           //
      (string(prefix_ + GROUP_OF_FRAMES_SIZE_OPT).c_str(),                   //
       value<size_t>(&groupOfFramesSize)->default_value(groupOfFramesSize),  //
       "groupOfFramesSize")                                                  //
      (string(prefix_ + OFFSET_OPT).c_str(),                                 //
       value<vector<double>>(&offset),                                       //
       "offset")                                                             //
      (string(prefix_ + QP_OPT).c_str(),                                     //
       value<int>(&qp)->default_value(qp),                                   //
       "qp")                                                                 //
      ;
  opts_.add(inputDataset.getOpts());
  opts_.add(inputReader.getOpts());
  opts_.add(outputDataset.getOpts());
}

void AppParameter::notify() {
  inputDataset.notify();
  inputReader.notify();
  outputDataset.notify(false);
  const filesystem::path& path = filesystem::path(outputDataset.getFilePath(0));
  if (!filesystem::exists(path.parent_path())) {
    filesystem::create_directories(path.parent_path());
  }
  THROW_IF_NOT(offset.size() == 0 || offset.size() == 3);
  if (!parallel) {
    groupOfFramesSize = 1;
  }
}

ostream& operator<<(ostream& out, const AppParameter& obj) {
  obj.coutParameters(out)                                //
      (PARALLEL_OPT, obj.parallel)                       //
      (GROUP_OF_FRAMES_SIZE_OPT, obj.groupOfFramesSize)  //
      (OFFSET_OPT, obj.offset)                           //
      (QP_OPT, obj.qp)                                   //
      ;
  out << obj.inputDataset;
  out << obj.inputReader;
  out << obj.outputDataset;
  return out;
}

}  // namespace jpcc