#pragma once

#include <string>

namespace jpcc {

enum class CoderBackendType { NONE = 0, TMC3 = 1 };

[[nodiscard]] CoderBackendType getCoderBackendType(const std::string& coderBackendType);
}  // namespace jpcc