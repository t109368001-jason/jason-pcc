#ifndef JPCC_COMMON_PARSER_PARAMETER_H_
#define JPCC_COMMON_PARSER_PARAMETER_H_

#include <iostream>
#include <string>
#include <vector>

#include <jpcc/common/Parameter.h>

namespace jpcc {
namespace common {

class ParserParameter : public Parameter {
 public:
  bool                     disableEnvVar;
  std::vector<std::string> configs;

  ParserParameter();

  std::string getConfigsOpt();

  friend std::ostream& operator<<(std::ostream& out, const ParserParameter& obj);
};

}  // namespace common
}  // namespace jpcc

#endif  // JPCC_COMMON_PARSER_PARAMETER_H_