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

std::ostream& operator<<(std::ostream& out, const CoderBackendType& obj) {
  switch (obj) {
    case CoderBackendType::NONE: out << "NONE"; break;
    case CoderBackendType::TMC3: out << "TMC3"; break;
    default: out << "Unknown"; break;
  }
  return out;
}

}  // namespace jpcc