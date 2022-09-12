#include <jpcc/segmentation/StaticPointType.h>

#include <stdexcept>

#include <boost/throw_exception.hpp>

using namespace std;

namespace jpcc::segmentation {

StaticPointType getStaticPointType(const string& staticPointType) {
  if (staticPointType == "center") {
    return StaticPointType::CENTER;
  } else {
    BOOST_THROW_EXCEPTION(logic_error("invalid staticPointType"));
  }
}

}  // namespace jpcc::segmentation