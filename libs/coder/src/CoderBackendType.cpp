#include <jpcc/coder/CoderBackendType.h>

using namespace std;

namespace jpcc::coder {

CoderBackendType getCoderBackendType(const std::string& coderBackendType) {
  if (coderBackendType == "none") {
    return CoderBackendType::NONE;
  } else if (coderBackendType == "tmc3") {
    return CoderBackendType::TMC3;
  } else {
    throw logic_error("invalid encoderBackendType");
  }
}

}  // namespace jpcc::coder