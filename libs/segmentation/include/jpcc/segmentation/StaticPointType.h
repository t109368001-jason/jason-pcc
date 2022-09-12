#pragma once

#include <string>

namespace jpcc::segmentation {

enum class StaticPointType { CENTER };

[[nodiscard]] StaticPointType getStaticPointType(const std::string& type);

}  // namespace jpcc::segmentation