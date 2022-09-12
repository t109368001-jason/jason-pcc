#pragma once

#include <jpcc/common/Common.h>

namespace jpcc {

enum class CoderBackendType { NONE, TMC3 };

[[nodiscard]] CoderBackendType getCoderBackendType(const std::string& coderBackendType);
}  // namespace jpcc