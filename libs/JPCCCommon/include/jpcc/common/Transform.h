#ifndef JPCC_COMMON_FRANSFORM_H_
#define JPCC_COMMON_FRANSFORM_H_

#include <math.h>

#include <jpcc/common/Common.h>

namespace jpcc {
namespace common {

constexpr auto PI_180 = M_PI / 180.0;

Point laser2Point(const Laser& laser);

}  // namespace common
}  // namespace jpcc

#endif  // JPCC_COMMON_FRANSFORM_H_