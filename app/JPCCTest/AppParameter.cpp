#include "AppParameter.h"

#include <boost/algorithm/string.hpp>

namespace jpcc {

using namespace std;
using namespace po;

#define APP_OPT_PREFIX "app"
#define PARALLEL_OPT_PREFIX ".parallel"
#define BOOL1_OPT_PREFIX ".bool1"
#define BOOL2_OPT_PREFIX ".bool2"
#define BOOL3_OPT_PREFIX ".bool3"
#define INT1_OPT_PREFIX ".int1"
#define INT2_OPT_PREFIX ".int2"
#define INT3_OPT_PREFIX ".int3"
#define FLOAT1_OPT_PREFIX ".float1"
#define FLOAT2_OPT_PREFIX ".float2"
#define FLOAT3_OPT_PREFIX ".float3"
#define STRING1_OPT_PREFIX ".string1"
#define STRING2_OPT_PREFIX ".string2"
#define STRING3_OPT_PREFIX ".string3"

AppParameter::AppParameter() :
    Parameter(APP_OPT_PREFIX, __FUNCTION__),
    parallel(false),
    dataset(),
    reader(),
    preProcess(),
    visualizerParameter(),
    bool1(false),
    bool2(false),
    bool3(false),
    int1(0),
    int2(0),
    int3(0),
    float1(NAN),
    float2(NAN),
    float3(NAN) {
  opts_.add_options()                                                                                               //
      (string(prefix_ + PARALLEL_OPT_PREFIX).c_str(), value<bool>(&parallel)->default_value(parallel), "parallel")  //
      (string(prefix_ + BOOL1_OPT_PREFIX).c_str(), value<bool>(&bool1)->default_value(bool1), "bool1")              //
      (string(prefix_ + BOOL2_OPT_PREFIX).c_str(), value<bool>(&bool2)->default_value(bool2), "bool2")              //
      (string(prefix_ + BOOL3_OPT_PREFIX).c_str(), value<bool>(&bool3)->default_value(bool3), "bool3")              //
      (string(prefix_ + INT1_OPT_PREFIX).c_str(), value<int>(&int1)->default_value(int1), "int1")                   //
      (string(prefix_ + INT2_OPT_PREFIX).c_str(), value<int>(&int2)->default_value(int2), "int2")                   //
      (string(prefix_ + INT3_OPT_PREFIX).c_str(), value<int>(&int3)->default_value(int3), "int3")                   //
      (string(prefix_ + FLOAT1_OPT_PREFIX).c_str(), value<float>(&float1)->default_value(float1), "float1")         //
      (string(prefix_ + FLOAT2_OPT_PREFIX).c_str(), value<float>(&float2)->default_value(float2), "float2")         //
      (string(prefix_ + FLOAT3_OPT_PREFIX).c_str(), value<float>(&float3)->default_value(float3), "float3")         //
      (string(prefix_ + STRING1_OPT_PREFIX).c_str(), value<string>(&string1)->default_value(string1), "string1")    //
      (string(prefix_ + STRING2_OPT_PREFIX).c_str(), value<string>(&string2)->default_value(string2), "string2")    //
      (string(prefix_ + STRING3_OPT_PREFIX).c_str(), value<string>(&string3)->default_value(string3), "string3")    //
      ;
  opts_.add(dataset.getOpts());
  opts_.add(reader.getOpts());
  opts_.add(preProcess.getOpts());
  opts_.add(visualizerParameter.getOpts());
}

void AppParameter::notify() {
  dataset.notify();
  reader.notify();
  preProcess.notify();
  visualizerParameter.notify();
}

ostream& operator<<(ostream& out, const AppParameter& obj) {
  out << obj.caption_ << endl;
  out << "\t" << obj.prefix_ << PARALLEL_OPT_PREFIX "=" << obj.parallel << endl;
  out << obj.dataset;
  out << obj.reader;
  out << obj.preProcess;
  out << obj.visualizerParameter;
  out << "\t" << obj.prefix_ << BOOL1_OPT_PREFIX "=" << obj.bool1 << endl;
  return out;
}

}  // namespace jpcc