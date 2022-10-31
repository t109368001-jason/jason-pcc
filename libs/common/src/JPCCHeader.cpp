#include <jpcc/common/JPCCHeader.h>

namespace jpcc {

//////////////////////////////////////////////////////////////////////////////////////////////
bool JPCCHeader::operator==(const JPCCHeader& other) const {
  if (this->resolution != other.resolution) { return false; }
  if (this->segmentationType != other.segmentationType) { return false; }
  if (this->segmentationOutputType != other.segmentationOutputType) { return false; }
  if (this->dynamicBackendType != other.dynamicBackendType) { return false; }
  if (this->staticBackendType != other.staticBackendType) { return false; }
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
std::ostream& writeJPCCHeader(const JPCCHeader& header, std::ostream& os) {
  os.write(reinterpret_cast<const char*>(&header.resolution), sizeof(header.resolution));
  os.put(char(header.segmentationType));
  os.put(char(header.segmentationOutputType));
  os.put(char(header.dynamicBackendType));
  os.put(char(header.staticBackendType));
  return os;
}

//////////////////////////////////////////////////////////////////////////////////////////////
std::istream& readJPCCHeader(std::istream& is, JPCCHeader* header) {
  is.read(reinterpret_cast<char*>(&header->resolution), sizeof(header->resolution));
  header->segmentationType       = SegmentationType(static_cast<char>(is.get()));
  header->segmentationOutputType = SegmentationOutputType(static_cast<char>(is.get()));
  header->dynamicBackendType     = CoderBackendType(static_cast<char>(is.get()));
  header->staticBackendType      = CoderBackendType(static_cast<char>(is.get()));
  return is;
}

}  // namespace jpcc