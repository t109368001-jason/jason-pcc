#pragma once

#undef NDEBUG
#include <cassert>
#include <memory>

#include <boost/throw_exception.hpp>

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>

namespace jpcc {

using std::shared_ptr;

using Point = pcl::PointXYZ;

}  // namespace jpcc
