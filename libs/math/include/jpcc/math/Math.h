#pragma once

#include <jpcc/common/Common.h>

namespace jpcc {

constexpr double TO_DEGREE_MULTIPLIER = 180.0 / M_PI;

constexpr double TO_RADIAN_MULTIPLIER = M_PI / 180.0;

template <class T>
double standard_deviation(const std::vector<T>& values);

template <class T>
double entropy(const std::vector<T>& values, T min, T max, T qp);

}  // namespace jpcc

#include <jpcc/math/impl/Math.hpp>
