#include <jpcc/metric/JPCCMetricParameter.h>

#include <filesystem>

namespace jpcc::metric {

using namespace std;
using namespace po;

#define OUTPUT_CSV_FOLDER_OPT ".outputCSVFolder"

JPCCMetricParameter::JPCCMetricParameter() : JPCCMetricParameter(JPCC_METRIC_OPT_PREFIX, __FUNCTION__) {}

JPCCMetricParameter::JPCCMetricParameter(const string& prefix, const string& caption) :
    Parameter(prefix, caption), outputCSVFolder() {
  opts_.add_options()                                    //
      (string(prefix_ + OUTPUT_CSV_FOLDER_OPT).c_str(),  //
       value<string>(&outputCSVFolder)->required(),      //
       "outputCSVPath")                                  //
      ;
}

void JPCCMetricParameter::notify() {
  outputCSVFolderPath = outputCSVFolder;
  if (!filesystem::exists(outputCSVFolderPath)) { create_directories(outputCSVFolderPath); }
}

ostream& operator<<(ostream& out, const JPCCMetricParameter& obj) {
  obj.coutParameters(out)                           //
      (OUTPUT_CSV_FOLDER_OPT, obj.outputCSVFolder)  //
      ;
  return out;
}

}  // namespace jpcc::metric
