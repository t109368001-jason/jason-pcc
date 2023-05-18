#include "AppParameter.h"

#include <filesystem>

namespace jpcc {

using namespace std;
using namespace po;

#define APP_OPT_PREFIX "app"
#define PARALLEL_OPT ".parallel"
#define GROUP_OF_FRAMES_SIZE_OPT ".groupOfFramesSize"
#define MAXIMUM_VALUE_OPT ".maximumValue"

AppParameter::AppParameter() :
    Parameter(APP_OPT_PREFIX, __FUNCTION__),
    parallel(false),
    groupOfFramesSize(32),
    maximumValue(100000),
    inputDataset("dataset", "InputDatasetParameter"),
    inputReader("reader", "InputDatasetReaderParameter"),
    reconstructDataset("reconstructDataset", "ReconstructDatasetParameter") {
  opts_.add_options()                                                        //
      (string(prefix_ + PARALLEL_OPT).c_str(),                               //
       value<bool>(&parallel)->default_value(parallel),                      //
       "parallel")                                                           //
      (string(prefix_ + GROUP_OF_FRAMES_SIZE_OPT).c_str(),                   //
       value<size_t>(&groupOfFramesSize)->default_value(groupOfFramesSize),  //
       "groupOfFramesSize")                                                  //
      (string(prefix_ + MAXIMUM_VALUE_OPT).c_str(),                          //
       value<double>(&maximumValue)->default_value(maximumValue),            //
       "maximumValue")                                                       //
      ;
  opts_.add(inputDataset.getOpts());
  opts_.add(inputReader.getOpts());
  opts_.add(reconstructDataset.getOpts());
}

void AppParameter::notify() {
  inputDataset.notify();
  inputReader.notify();
  reconstructDataset.notify();
  if (!parallel) {
    groupOfFramesSize = 1;
  }
}

ostream& operator<<(ostream& out, const AppParameter& obj) {
  obj.coutParameters(out)                                //
      (PARALLEL_OPT, obj.parallel)                       //
      (GROUP_OF_FRAMES_SIZE_OPT, obj.groupOfFramesSize)  //
      (MAXIMUM_VALUE_OPT, obj.maximumValue)              //
      ;
  out << obj.inputDataset;
  out << obj.inputReader;
  out << obj.reconstructDataset;
  return out;
}

}  // namespace jpcc