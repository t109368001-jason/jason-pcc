#include "AppParameter.h"

#include <filesystem>

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
    compressedStreamPath("./bin/output.bin"),
    dataset("dataset", "InputDatasetParameter"),
    reader("reader", "InputDatasetReaderParameter"),
    preProcess(),
    jpccGmmSegmentation(),
    jpccEncoderDynamic("jpccEncoderDynamic", "JPCCEncoderTMC3Dynamic"),
    jpccEncoderStatic("jpccEncoderStatic", "JPCCEncoderTMC3Static") {
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
  opts_.add(jpccGmmSegmentation.getOpts());
  opts_.add(jpccEncoderDynamic.getOpts());
  opts_.add(jpccEncoderStatic.getOpts());
  opts_.add(normalEstimation.getOpts());
  opts_.add(metricParameter.getOpts());
}

void AppParameter::notify() {
  dataset.notify();
  reader.notify();
  preProcess.notify();
  jpccGmmSegmentation.notify();
  jpccEncoderDynamic.notify();
  jpccEncoderStatic.notify();
  normalEstimation.notify();
  metricParameter.notify();
  if (!parallel) { groupOfFramesSize = 1; }
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
  out << obj.jpccGmmSegmentation;
  out << obj.jpccEncoderDynamic;
  out << obj.jpccEncoderStatic;
  out << obj.normalEstimation;
  out << obj.metricParameter;
  return out;
}

}  // namespace jpcc