#include <jpcc/common/SegmentationOutputType.h>

#include <stdexcept>

#include <boost/throw_exception.hpp>

using namespace std;

namespace jpcc {

SegmentationOutputType getSegmentationOutputType(const string& segmentationOutputType) {
  if (segmentationOutputType == "none") {
    return SegmentationOutputType::NONE;
  } else if (segmentationOutputType == "dynamic-static") {
    return SegmentationOutputType::DYNAMIC_STATIC;
  } else if (segmentationOutputType == "dynamic-staticAdded-staticRemoved") {
    return SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED;
  } else {
    BOOST_THROW_EXCEPTION(logic_error("invalid segmentationOutputType"));
  }
}

std::ostream& operator<<(std::ostream& out, const SegmentationOutputType& obj) {
  switch (obj) {
    case SegmentationOutputType::NONE: out << "NONE"; break;
    case SegmentationOutputType::DYNAMIC_STATIC: out << "DYNAMIC_STATIC"; break;
    case SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED:
      out << "DYNAMIC_STATIC_ADDED_STATIC_REMOVED";
      break;
    default: out << "Unknown"; break;
  }
  return out;
}

}  // namespace jpcc