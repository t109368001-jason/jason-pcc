#include <jpcc/encoder/JPCCEncoderParameter.h>

namespace jpcc::encoder {

using namespace std;
using namespace po;

#define BACKEND_TYPE_OPT ".backendType"

JPCCEncoderParameter::JPCCEncoderParameter() : JPCCEncoderParameter(JPCC_ENCODER_OPT_PREFIX, __FUNCTION__) {
}

JPCCEncoderParameter::JPCCEncoderParameter(const string& prefix, const string& caption) :
    Parameter(prefix, caption),
    backendType_("none"),
    backendType(CoderBackendType::NONE),
#if defined(HAVE_MPEG_PCC_TMC2)
    tmc2(prefix + ".tmc2", "JPCCEncoderTMC2Parameter"),
#endif
    tmc3(prefix + ".tmc3", "JPCCEncoderTMC3Parameter") {
  opts_.add_options()                               //
      (string(prefix_ + BACKEND_TYPE_OPT).c_str(),  //
       value<string>(&backendType_)->required(),    //
       "backendType")                               //
      ;
#if defined(HAVE_MPEG_PCC_TMC2)
  opts_.add(tmc2.getOpts());
#endif
  opts_.add(tmc3.getOpts());
}

void JPCCEncoderParameter::notify() {
  backendType = getCoderBackendType(backendType_);
#if defined(HAVE_MPEG_PCC_TMC2)
  if (backendType == CoderBackendType::TMC2) {
    tmc2.notify();
  }
#else
  THROW_IF_NOT(backendType != CoderBackendType::TMC2);
#endif
  if (backendType == CoderBackendType::TMC3) {
    tmc3.notify();
  }
}

ostream& operator<<(ostream& out, const JPCCEncoderParameter& obj) {
  obj.coutParameters(out)                   //
      (BACKEND_TYPE_OPT, obj.backendType_)  //
      ;
#if defined(HAVE_MPEG_PCC_TMC2)
  if (obj.backendType == CoderBackendType::TMC2) {
    out << obj.tmc2;
  }
#endif
  if (obj.backendType == CoderBackendType::TMC3) {
    out << obj.tmc3;
  }
  return out;
}

}  // namespace jpcc::encoder
