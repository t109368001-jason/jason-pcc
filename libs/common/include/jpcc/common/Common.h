#ifndef JPCC_COMMON_COMMON_H_
#define JPCC_COMMON_COMMON_H_

#undef NDEBUG
#include <cassert>
#include <memory>

#include <boost/throw_exception.hpp>

#include <pcl/point_types.h>

namespace jpcc {

using std::shared_ptr;

using Point = pcl::PointXYZ;

}  // namespace jpcc

#endif  // JPCC_COMMON_COMMON_H_