#include <jpcc/coder/JPCCDecoderTMC3Parameter.h>

namespace jpcc::coder {

using namespace std;
using namespace po;
using namespace pcc;

#define BACKEND_TYPE_OPT ".backendType"

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCDecoderTMC3Parameter::JPCCDecoderTMC3Parameter() :
    JPCCDecoderTMC3Parameter(JPCC_DECODER_TMC3_OPT_PREFIX, __FUNCTION__) {}

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCDecoderTMC3Parameter::JPCCDecoderTMC3Parameter(const string& prefix, const string& caption) :
    Parameter(prefix, caption), DecoderParams() {
  setDefault();
  opts_.add_options()  //
                       // TODO
      ;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCDecoderTMC3Parameter::setDefault() {
  // TODO
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCDecoderTMC3Parameter::notify() {}

//////////////////////////////////////////////////////////////////////////////////////////////
ostream& operator<<(ostream& out, const JPCCDecoderTMC3Parameter& obj) {
  obj.coutParameters(out)  //
      ;
  return out;
}

}  // namespace jpcc::coder
