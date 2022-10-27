#pragma once

#include <string>

namespace jpcc {

enum class SegmentationOutputType { NONE, DYNAMIC_STATIC, DYNAMIC_STATIC_ADDED_STATIC_REMOVED };

[[nodiscard]] SegmentationOutputType getSegmentationOutputType(const std::string& type);

}  // namespace jpcc