#include <jpcc/common/Common.h>

namespace jpcc {

using namespace std;

ostream& operator<<(ostream& out, const PointNormal& p) {
  out << "(" << p.x << "," << p.y << "," << p.z << " - " << p.normal[0] << "," << p.normal[1] << "," << p.normal[2]
      << " - " << p.curvature << ")";
  return out;
}

}  // namespace jpcc
