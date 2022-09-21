#pragma once

#include <string>

namespace jpcc {

enum class SegmentationType { NONE, GMM, GMM_2L };

[[nodiscard]] SegmentationType getSegmentationType(const std::string& type);

}  // namespace jpcc