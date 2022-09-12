#pragma once

#include <string>

namespace jpcc {

enum class SegmentationType { GMM, GMM_2L };

[[nodiscard]] SegmentationType getSegmentationType(const std::string& type);

}  // namespace jpcc