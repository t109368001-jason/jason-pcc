#include <jpcc/common/SegmentationType.h>

#include <stdexcept>

#include <boost/throw_exception.hpp>

using namespace std;

namespace jpcc {

SegmentationType getSegmentationType(const string& segmentationType) {
  if (segmentationType == "none") {
    return SegmentationType::NONE;
  } else if (segmentationType == "gmm") {
    return SegmentationType::GMM;
  } else if (segmentationType == "gmm-2l") {
    return SegmentationType::GMM_2L;
  } else {
    BOOST_THROW_EXCEPTION(logic_error("invalid segmentationType"));
  }
}

std::ostream& operator<<(std::ostream& out, const SegmentationType& obj) {
  switch (obj) {
    case SegmentationType::NONE: out << "NONE"; break;
    case SegmentationType::GMM: out << "GMM"; break;
    case SegmentationType::GMM_2L: out << "GMM_2L"; break;
    default: out << "Unknown"; break;
  }
  return out;
}

}  // namespace jpcc