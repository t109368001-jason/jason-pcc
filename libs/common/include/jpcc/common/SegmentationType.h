#pragma once

#include <string>

namespace jpcc {

enum class SegmentationType { NONE = 0, GMM = 1, GMM_2L = 2 };

[[nodiscard]] SegmentationType getSegmentationType(const std::string& type);

}  // namespace jpcc