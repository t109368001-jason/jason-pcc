#pragma once

#include <set>
#include <string>
#include <vector>

#include <boost/program_options.hpp>

#include <jpcc/common/Parameter.h>
#include <jpcc/common/ParserParameter.h>

namespace jpcc {

class ParameterParser {
 protected:
  po::options_description         opts_;
  ParserParameter                 param_;
  std::vector<Parameter*>         params_;
  std::vector<po::parsed_options> pOpts_;
  std::set<std::string>           pCfgs_;  // parsed configs

 public:
  ParameterParser();

  void add(Parameter& param);

  [[nodiscard]] bool parse(int argc, char* argv[]);

  void parseConfigs(const po::variables_map& vm);

  static void conflicting_options(const po::variables_map& vm, const std::string& opt1, const std::string& opt2);

  static void option_dependency(const po::variables_map& vm, const std::string& opt, const std::string& requiredOpt);
};

}  // namespace jpcc
