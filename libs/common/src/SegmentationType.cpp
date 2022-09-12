#include <jpcc/common/SegmentationType.h>

#include <stdexcept>

#include <boost/throw_exception.hpp>

using namespace std;

namespace jpcc {

SegmentationType getSegmentationType(const string& segmentationType) {
  if (segmentationType == "gmm") {
    return SegmentationType::GMM;
  } else if (segmentationType == "gmm-2l") {
    return SegmentationType::GMM_2L;
  } else {
    BOOST_THROW_EXCEPTION(logic_error("invalid segmentationType"));
  }
}

}  // namespace jpcc