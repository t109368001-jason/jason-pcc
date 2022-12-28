#include <jpcc/encoder/JPCCEncoderTMC2Parameter.h>

namespace jpcc::encoder {

using namespace std;
using namespace po;
using namespace pcc;

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCEncoderTMC2Parameter::JPCCEncoderTMC2Parameter() :
    JPCCEncoderTMC2Parameter(JPCC_ENCODER_TMC2_OPT_PREFIX, __FUNCTION__) {}

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCEncoderTMC2Parameter::JPCCEncoderTMC2Parameter(  // NOLINT(cppcoreguidelines-pro-type-member-init)
    const string& prefix,
    const string& caption) :
    Parameter(prefix, caption) {
  setDefault();
  opts_.add_options()  //
                       // TODO
      ;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoderTMC2Parameter::setDefault() {
  // TODO
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoderTMC2Parameter::notify() {}

//////////////////////////////////////////////////////////////////////////////////////////////
ostream& operator<<(ostream& out, const JPCCEncoderTMC2Parameter& obj) {
  obj.coutParameters(out)  //
                           // TODO
      ;
  return out;
}

}  // namespace jpcc::encoder
