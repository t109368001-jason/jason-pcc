#include <jpcc/common/JPCCHeader.h>

namespace jpcc {

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCHeader::write(std::ostream& os) const {
  os.write(reinterpret_cast<const char*>(&resolution), sizeof(resolution));
  os.put(char(backendType));
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCHeader::read(std::istream& is) {
  is.read(reinterpret_cast<char*>(&resolution), sizeof(resolution));
  backendType = CoderBackendType(static_cast<char>(is.get()));
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool JPCCHeader::operator==(const JPCCHeader& other) const {
  if (this->resolution != other.resolution) {
    return false;
  }
  if (this->backendType != other.backendType) {
    return false;
  }
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
std::ostream& operator<<(std::ostream& out, const JPCCHeader& obj) {
  out << "JPCCHeader(";
  out << "resolution=" << obj.resolution << ",";
  out << "backendType=" << obj.backendType;
  out << ")";
  return out;
}

}  // namespace jpcc