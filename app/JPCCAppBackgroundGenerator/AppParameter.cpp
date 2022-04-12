#include "AppParameter.h"

#include <boost/algorithm/string.hpp>

namespace jpcc {

using namespace std;
using namespace po;

#define APP_OPT_PREFIX "app"
#define PARALLEL_OPT ".parallel"
#define GROUP_OF_FRAMES_SIZE_OPT ".groupOfFramesSize"

AppParameter::AppParameter() :
    Parameter(APP_OPT_PREFIX, __FUNCTION__),
    parallel(false),
    groupOfFramesSize(32),
    dataset(),
    reader(),
    preProcess(),
    jpccNormalEstimation() {
  opts_.add_options()                                                        //
      (string(prefix_ + PARALLEL_OPT).c_str(),                               //
       value<bool>(&parallel)->default_value(parallel),                      //
       "parallel")                                                           //
      (string(prefix_ + GROUP_OF_FRAMES_SIZE_OPT).c_str(),                   //
       value<size_t>(&groupOfFramesSize)->default_value(groupOfFramesSize),  //
       "groupOfFramesSize")                                                  //
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

ostream& operator<<(ostream& out, const AppParameter& obj) {
  out << obj.caption_ << endl;
  out << "\t" << obj.prefix_ << PARALLEL_OPT "=" << obj.parallel << endl;
  out << "\t" << obj.prefix_ << GROUP_OF_FRAMES_SIZE_OPT "=" << obj.groupOfFramesSize << endl;
  out << obj.dataset;
  out << obj.reader;
  out << obj.preProcess;
  out << obj.jpccNormalEstimation;
  return out;
}

}  // namespace jpcc