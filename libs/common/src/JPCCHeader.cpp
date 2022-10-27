#include <jpcc/common/JPCCHeader.h>

namespace jpcc {

std::ostream& writeJPCCHeader(const JPCCHeader& header, std::ostream& os) {
  os.put(int8_t(header.segmentationType));
  os.put(int8_t(header.segmentationOutputType));
  os.put(int8_t(header.dynamicBackendType));
  os.put(int8_t(header.staticBackendType));
  return os;
}

std::istream& readJPCCHeader(std::istream& is, JPCCHeader* header) {
  header->segmentationType       = SegmentationType(static_cast<int8_t>(is.get()));
  header->segmentationOutputType = SegmentationOutputType(static_cast<int8_t>(is.get()));
  header->dynamicBackendType     = CoderBackendType(static_cast<int8_t>(is.get()));
  header->staticBackendType      = CoderBackendType(static_cast<int8_t>(is.get()));
  return is;
}

}  // namespace jpcc