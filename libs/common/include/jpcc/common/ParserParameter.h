#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <jpcc/common/Parameter.h>

namespace jpcc::common {

class ParserParameter : public Parameter {
 public:
  bool                     disableEnvVar;
  std::vector<std::string> configs;

  ParserParameter();

  [[nodiscard]] static std::string getConfigsOpt();

  friend std::ostream& operator<<(std::ostream& out, const ParserParameter& obj);
};

}  // namespace jpcc::common
