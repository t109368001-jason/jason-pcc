#include <jpcc/common/Common.h>

namespace jpcc {

std::ostream& operator<<(std::ostream& os, const PointNormal& p) {
  os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.normal[0] << "," << p.normal[1] << "," << p.normal[2]
     << " - " << p.curvature << ")";
  return (os);
}

}  // namespace jpcc
