#pragma once

#include <type_traits>

namespace jpcc {

template <class T>
struct dependent_false : std::false_type {};

template <class T>
inline constexpr bool dependent_false_v = dependent_false<T>::value;

}  // namespace jpcc
