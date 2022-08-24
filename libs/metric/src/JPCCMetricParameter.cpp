#include <jpcc/metric/JPCCMetricParameter.h>

#include <filesystem>

namespace jpcc::metric {

using namespace std;
using namespace po;

#define OUTPUT_CSV_FOLDER_OPT ".outputCSVFolder"
#define MAXIMUM_VALUE_OPT ".maximumValue"

JPCCMetricParameter::JPCCMetricParameter() : JPCCMetricParameter(JPCC_METRIC_OPT_PREFIX, __FUNCTION__) {}

JPCCMetricParameter::JPCCMetricParameter(const string& prefix, const string& caption) :
    Parameter(prefix, caption), outputCSVFolder(), maximumValue(100.0) {
  opts_.add_options()                                    //
      (string(prefix_ + OUTPUT_CSV_FOLDER_OPT).c_str(),  //
       value<string>(&outputCSVFolder)->required(),      //
       "outputCSVPath")                                  //
      (string(prefix_ + MAXIMUM_VALUE_OPT).c_str(),      //
       value<double>(&maximumValue)->required(),         //
       "maximumValue")                                   //
      ;
}

void JPCCMetricParameter::notify() {
  outputCSVFolderPath = outputCSVFolder;
  if (!filesystem::exists(outputCSVFolderPath)) { create_directories(outputCSVFolderPath); }
  THROW_IF_NOT(maximumValue > 0);
}

ostream& operator<<(ostream& out, const JPCCMetricParameter& obj) {
  obj.coutParameters(out)                           //
      (OUTPUT_CSV_FOLDER_OPT, obj.outputCSVFolder)  //
      ;
  return out;
}

}  // namespace jpcc::metric
