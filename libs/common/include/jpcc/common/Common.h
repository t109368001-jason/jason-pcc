#pragma once

#include <cassert>

#include <boost/throw_exception.hpp>

#include <jpcc/common/JPCCPointSet3.h>
#include <jpcc/common/Traits.h>
#include <jpcc/common/Types.h>

#define JPCC_NOT_USED(x)

#define THROW_IF_NOT(expression)                                                 \
  do {                                                                           \
    if (!(expression)) { BOOST_THROW_EXCEPTION(std::logic_error(#expression)); } \
  } while (0)

namespace jpcc {

constexpr float MAX_INTENSITY  = 1.0;
constexpr float NULL_INTENSITY = -0.5;

}  // namespace jpcc
