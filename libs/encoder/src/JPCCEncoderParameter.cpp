#include <jpcc/encoder/JPCCEncoderParameter.h>

namespace jpcc::encoder {

using namespace std;
using namespace po;

#define BACKEND_TYPE_OPT ".backendType"

JPCCEncoderParameter::JPCCEncoderParameter() : JPCCEncoderParameter(JPCC_ENCODER_OPT_PREFIX, __FUNCTION__) {}

JPCCEncoderParameter::JPCCEncoderParameter(const string& prefix, const string& caption) :
    Parameter(prefix, caption),
    backendType_("none"),
    backendType(EncoderBackendType::NONE),
    tmc3("tmc3", "JPCCEncoderTMC3Parameter") {
  opts_.add_options()                               //
      (string(prefix_ + BACKEND_TYPE_OPT).c_str(),  //
       value<string>(&backendType_)->required(),    //
       "backendType")                               //
      ;
}

void JPCCEncoderParameter::notify() { backendType = getEncoderBackendType(backendType_); }

ostream& operator<<(ostream& out, const JPCCEncoderParameter& obj) {
  obj.coutParameters(out)                   //
      (BACKEND_TYPE_OPT, obj.backendType_)  //
      ;
  return out;
}

EncoderBackendType getEncoderBackendType(const std::string& encoderBackendType) {
  if (encoderBackendType == "none") {
    return EncoderBackendType::NONE;
  } else if (encoderBackendType == "tmc3") {
    return EncoderBackendType::TMC3;
  } else {
    throw logic_error("invalid encoderBackendType");
  }
}

}  // namespace jpcc::encoder
