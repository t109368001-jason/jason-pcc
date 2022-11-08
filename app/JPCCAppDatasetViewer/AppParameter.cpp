#include "AppParameter.h"

#include <filesystem>

#include <boost/algorithm/string.hpp>

namespace jpcc {

using namespace std;
using namespace po;

#define APP_OPT_PREFIX "app"
#define PARALLEL_OPT ".parallel"
#define GROUP_OF_FRAMES_SIZE_OPT ".groupOfFramesSize"
#define COMPRESSED_DYNAMIC_STREAM_PATH_OPT ".compressedStreamPath"

AppParameter::AppParameter() :
    Parameter(APP_OPT_PREFIX, __FUNCTION__),
    parallel(false),
    groupOfFramesSize(32),
    compressedStreamPath(),
    dataset(),
    reader(),
    preProcess() {
  opts_.add_options()                                                              //
      (string(prefix_ + PARALLEL_OPT).c_str(),                                     //
       value<bool>(&parallel)->default_value(parallel),                            //
       "parallel")                                                                 //
      (string(prefix_ + GROUP_OF_FRAMES_SIZE_OPT).c_str(),                         //
       value<size_t>(&groupOfFramesSize)->default_value(groupOfFramesSize),        //
       "groupOfFramesSize")                                                        //
      (string(prefix_ + COMPRESSED_DYNAMIC_STREAM_PATH_OPT).c_str(),               //
       value<string>(&compressedStreamPath)->default_value(compressedStreamPath),  //
       "compressedStreamPath")                                                     //
      ;
  opts_.add(dataset.getOpts());
  opts_.add(reader.getOpts());
  opts_.add(preProcess.getOpts());
  opts_.add(visualizerParameter.getOpts());
}

void AppParameter::getShowTexts(vector<std::string>& showTexts) const {
  dataset.getShowTexts(showTexts);
  reader.getShowTexts(showTexts);
  preProcess.getShowTexts(showTexts);
  visualizerParameter.getShowTexts(showTexts);
}

void AppParameter::notify() {
  dataset.notify();
  reader.notify();
  preProcess.notify();
  visualizerParameter.notify();
  if (!parallel) { groupOfFramesSize = 1; }
  if (!compressedStreamPath.empty()) { THROW_IF_NOT(filesystem::exists(compressedStreamPath)); }
}

ostream& operator<<(ostream& out, const AppParameter& obj) {
  obj.coutParameters(out)                                             //
      (PARALLEL_OPT, obj.parallel)                                    //
      (GROUP_OF_FRAMES_SIZE_OPT, obj.groupOfFramesSize)               //
      (COMPRESSED_DYNAMIC_STREAM_PATH_OPT, obj.compressedStreamPath)  //
      ;
  out << obj.dataset;
  out << obj.reader;
  out << obj.preProcess;
  out << obj.visualizerParameter;
  return out;
}

}  // namespace jpcc