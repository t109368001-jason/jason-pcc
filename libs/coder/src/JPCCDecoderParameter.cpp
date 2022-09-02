#include <jpcc/coder/JPCCDecoderParameter.h>

namespace jpcc::coder {

using namespace std;
using namespace po;

#define BACKEND_TYPE_OPT ".backendType"

JPCCDecoderParameter::JPCCDecoderParameter() : JPCCDecoderParameter(JPCC_DECODER_OPT_PREFIX, __FUNCTION__) {}

JPCCDecoderParameter::JPCCDecoderParameter(const string& prefix, const string& caption) :
    Parameter(prefix, caption),
    backendType_("none"),
    backendType(CoderBackendType::NONE),
    tmc3("tmc3", "JPCCDecoderTMC3Parameter") {
  opts_.add_options()                               //
      (string(prefix_ + BACKEND_TYPE_OPT).c_str(),  //
       value<string>(&backendType_)->required(),    //
       "backendType")                               //
      ;
}

void JPCCDecoderParameter::notify() { backendType = getCoderBackendType(backendType_); }

ostream& operator<<(ostream& out, const JPCCDecoderParameter& obj) {
  obj.coutParameters(out)                   //
      (BACKEND_TYPE_OPT, obj.backendType_)  //
      ;
  return out;
}

}  // namespace jpcc::coder
