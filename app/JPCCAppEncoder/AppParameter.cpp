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
    inputDataset("dataset", "InputDatasetParameter"),
    inputReader("reader", "InputDatasetReaderParameter"),
    outputDataset("outputDataset", "OutputDatasetParameter"),
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
  opts_.add(inputDataset.getOpts());
  opts_.add(inputReader.getOpts());
  opts_.add(outputDataset.getOpts());
  opts_.add(preProcess.getOpts());
  opts_.add(jpccGmmSegmentation.getOpts());
  opts_.add(jpccEncoderDynamic.getOpts());
  opts_.add(jpccEncoderStatic.getOpts());
  opts_.add(normalEstimation.getOpts());
  opts_.add(metricParameter.getOpts());
}

void AppParameter::notify() {
  inputDataset.notify();
  inputReader.notify();
  outputDataset.notify(false);
  preProcess.notify();
  jpccGmmSegmentation.notify();
  jpccEncoderDynamic.notify();
  jpccEncoderStatic.notify();
  normalEstimation.notify();
  metricParameter.notify();
  if (!parallel) { groupOfFramesSize = 1; }
  const filesystem::path& path = filesystem::path(outputDataset.getFilePath(0));
  if (!filesystem::exists(path.parent_path())) { filesystem::create_directories(path.parent_path()); }
}

ostream& operator<<(ostream& out, const AppParameter& obj) {
  obj.coutParameters(out)                                             //
      (PARALLEL_OPT, obj.parallel)                                    //
      (GROUP_OF_FRAMES_SIZE_OPT, obj.groupOfFramesSize)               //
      (COMPRESSED_DYNAMIC_STREAM_PATH_OPT, obj.compressedStreamPath)  //
      ;
  out << obj.inputDataset;
  out << obj.inputReader;
  out << obj.outputDataset;
  out << obj.preProcess;
  out << obj.jpccGmmSegmentation;
  out << obj.jpccEncoderDynamic;
  out << obj.jpccEncoderStatic;
  out << obj.normalEstimation;
  out << obj.metricParameter;
  return out;
}

}  // namespace jpcc