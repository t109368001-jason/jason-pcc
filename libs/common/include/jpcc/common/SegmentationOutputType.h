#pragma once

#include <iostream>
#include <string>

namespace jpcc {

enum class SegmentationOutputType { NONE = 0, DYNAMIC_STATIC = 1, DYNAMIC_STATIC_ADDED_STATIC_REMOVED = 2 };

[[nodiscard]] SegmentationOutputType getSegmentationOutputType(const std::string& type);

std::ostream& operator<<(std::ostream& out, const SegmentationOutputType& obj);

}  // namespace jpcc