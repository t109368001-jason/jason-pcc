#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <jpcc/common/Parameter.h>

namespace jpcc {

class ParserParameter : public virtual Parameter {
 public:
  bool                     disableEnvVar;
  std::vector<std::string> configs;

  ParserParameter();

  ParserParameter(const std::string& prefix, const std::string& caption);

  [[nodiscard]] static std::string getConfigsOpt();

  [[nodiscard]] static std::string getConfigConfigsOpt();

  friend std::ostream& operator<<(std::ostream& out, const ParserParameter& obj);
};

}  // namespace jpcc
