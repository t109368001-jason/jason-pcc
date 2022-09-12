#pragma once

#include <string>

namespace jpcc::segmentation {

enum class SegmentationType { GMM, GMM_2L };

[[nodiscard]] SegmentationType getSegmentationType(const std::string& type);

}  // namespace jpcc::segmentation