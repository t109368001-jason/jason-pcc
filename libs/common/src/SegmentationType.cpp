#include <jpcc/common/SegmentationType.h>

#include <stdexcept>

using namespace std;

namespace jpcc {

SegmentationType getSegmentationType(const string& segmentationType) {
  if (segmentationType == "gmm") {
    return SegmentationType::GMM;
  } else if (segmentationType == "gmm-2l") {
    return SegmentationType::GMM_2L;
  } else {
    throw logic_error("invalid segmentationType");
  }
}

}  // namespace jpcc