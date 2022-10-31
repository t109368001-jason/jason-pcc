#pragma once

#include <iostream>
#include <string>

namespace jpcc {

enum class SegmentationType : uint8_t { NONE = 0, GMM = 1, GMM_2L = 2 };

[[nodiscard]] SegmentationType getSegmentationType(const std::string& type);

std::ostream& operator<<(std::ostream& out, const SegmentationType& obj);

}  // namespace jpcc