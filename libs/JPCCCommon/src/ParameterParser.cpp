#include <jpcc/common/ParameterParser.h>

#include <algorithm>
#include <fstream>

#include <boost/program_options/cmdline.hpp>
#include <boost/program_options/config.hpp>

namespace jpcc {
namespace common {

using namespace std;
using namespace po;

ParameterParser::ParameterParser() : opts_("Options") {
  opts_.add_options()             //
      ("help,h", "Help message")  //
      ;
  opts_.add(param_.getOpts());
}

void ParameterParser::add(Parameter& param) {
  params_.push_back(&param);
  opts_.add(param.getOpts());
}

void ParameterParser::parse(int argc, char* argv[]) {
  const parsed_options cmdOpts = parse_command_line(argc, argv, opts_);
  pOpts_.push_back(cmdOpts);
  variables_map vm;
  store(cmdOpts, vm);
  if (vm.count("help")) {
    cout << opts_ << "\n";
    return;
  }
  notify(vm);
  std::cout << param_ << std::endl;
  const parsed_options envOpts =
      parse_environment(opts_, boost::function1<string, string>([&](string envKey) {
                          transform(envKey.begin(), envKey.end(), envKey.begin(), ::toupper);
                          return opts_.find_nothrow(envKey, false, false, false) ? envKey : "";
                        }));
  pOpts_.push_back(envOpts);
  store(envOpts, vm);
  if (vm.count("config")) { parseConfigs(vm["config"].as<vector<string>>()); }

  variables_map vm_final;
  for_each(pOpts_.begin(), pOpts_.end(), [&vm_final](auto& pOpts) { store(pOpts, vm_final); });

  for (Parameter* param : params_) {
    for (auto& [p1, p2] : param->getConflicts()) { conflicting_options(vm_final, p1, p2); }
    for (auto& [p1, p2] : param->getDependencies()) { option_dependency(vm_final, p1, p2); }
  }

  notify(vm_final);
}

void ParameterParser::parseConfigs(const std::vector<std::string>& configs) {
  // latest config have higher priority
  for (auto it = configs.rbegin(); it != configs.rend(); ++it) {
    const string& config = *it;
    // check if config parsed then return
    if (pCfgs_.find(config) != pCfgs_.end()) { return; }
    ifstream ifs(config.c_str());
    if (!ifs.good()) { throw runtime_error(string("config: '") + config + "' not found"); }
    const parsed_options cfgOpts = parse_config_file(ifs, opts_, true);
    variables_map        vm;
    store(cfgOpts, vm);
    pOpts_.push_back(cfgOpts);
    pCfgs_.insert(config);
    if (vm.count("config")) { parseConfigs(vm["config"].as<vector<string>>()); }
  }
}

void ParameterParser::conflicting_options(const po::variables_map& vm,
                                          const std::string&       opt1,
                                          const std::string&       opt2) {
  if (vm.count(opt1) && !vm[opt1].defaulted() && vm.count(opt2) && !vm[opt2].defaulted())
    throw logic_error(string("Conflicting options '") + opt1 + "' and '" + opt2 + "'.");
}

void ParameterParser::option_dependency(const po::variables_map& vm,
                                        const std::string&       opt,
                                        const std::string&       requiredOpt) {
  if (vm.count(opt) && !vm[opt].defaulted())
    if (vm.count(requiredOpt) == 0 || vm[requiredOpt].defaulted())
      throw logic_error(string("Option '") + opt + "' requires option '" + requiredOpt + "'.");
}

}  // namespace common
}  // namespace jpcc
