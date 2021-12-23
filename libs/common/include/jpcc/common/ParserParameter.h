#ifndef JPCC_COMMON_PARSER_PARAMETER_H_
#define JPCC_COMMON_PARSER_PARAMETER_H_

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

#endif  // JPCC_COMMON_PARSER_PARAMETER_H_