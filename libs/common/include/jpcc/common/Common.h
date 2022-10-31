#pragma once

#include <cassert>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <type_traits>

#include <boost/throw_exception.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <PCCChrono.h>
#include <PCCMemory.h>

#include <jpcc/common/JPCCPointSet3.h>

#define JPCC_NOT_USED(x)

#define THROW_IF_NOT(expression)                                                 \
  do {                                                                           \
    if (!(expression)) { BOOST_THROW_EXCEPTION(std::logic_error(#expression)); } \
  } while (0)

namespace jpcc {

using std::make_shared;
using std::shared_ptr;

using index_t    = pcl::index_t;
using uindex_t   = pcl::uindex_t;
using Indices    = pcl::Indices;
using IndicesPtr = shared_ptr<Indices>;

using Frame         = JPCCPointSet3;
using FramePtr      = shared_ptr<Frame>;
using FrameConstPtr = shared_ptr<const Frame>;

using GroupOfFrame    = std::vector<FramePtr>;
using GroupOfFrameMap = std::map<std::string, GroupOfFrame>;

using Stopwatch = pcc::chrono::Stopwatch<std::chrono::steady_clock>;

template <class T>
struct dependent_false : std::false_type {};

template <class T>
inline constexpr bool dependent_false_v = dependent_false<T>::value;

constexpr float MAX_INTENSITY  = 1.0;
constexpr float NULL_INTENSITY = -0.5;

}  // namespace jpcc
