#pragma once

#include <iostream>
#include <string>

namespace jpcc {

enum class CoderBackendType { NONE = 0, TMC3 = 1 };

[[nodiscard]] CoderBackendType getCoderBackendType(const std::string& coderBackendType);

std::ostream& operator<<(std::ostream& out, const CoderBackendType& obj);

}  // namespace jpcc