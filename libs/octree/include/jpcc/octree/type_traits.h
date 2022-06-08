#include <type_traits>

template <typename, typename, typename = void>
struct has_add_point : std::false_type {};

template <typename T, typename PointT>
struct has_add_point<T, PointT, std::void_t<decltype(&T::addPoint)>>
    : std::is_same<void, decltype(std::declval<T>().addPoint(std::declval<const PointT&>()))> {};

template <class T, class PointT>
inline constexpr bool has_add_point_v = has_add_point<T, PointT>::value;
