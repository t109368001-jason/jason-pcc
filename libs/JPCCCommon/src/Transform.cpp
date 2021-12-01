#include <jpcc/common/Transform.h>

namespace jpcc {
namespace common {

Point laser2Point(const Laser& laser) {
  float phi   = laser.azimuth * PI_180;
  float theta = -(laser.vertical * PI_180) + 90.0;
  float x     = static_cast<float>(laser.distance * cos(phi) * sin(theta));
  float y     = static_cast<float>(laser.distance * sin(phi) * sin(theta));
  float z     = static_cast<float>(laser.distance * cos(theta));
  return Point(x, y, z);
}

}  // namespace common
}  // namespace jpcc
