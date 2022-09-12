#include <jpcc/common/CoderBackendType.h>

#include <stdexcept>

#include <boost/throw_exception.hpp>

using namespace std;

namespace jpcc {

CoderBackendType getCoderBackendType(const std::string& coderBackendType) {
  if (coderBackendType == "none") {
    return CoderBackendType::NONE;
  } else if (coderBackendType == "tmc3") {
    return CoderBackendType::TMC3;
  } else {
    BOOST_THROW_EXCEPTION(logic_error("invalid encoderBackendType"));
  }
}

}  // namespace jpcc