#include <jpcc/common/ParameterParser.h>

#include <boost/log/trivial.hpp>

#include <algorithm>
#include <fstream>

namespace jpcc {

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

bool ParameterParser::parse(int argc, char* argv[]) {
  try {
    const parsed_options cmdOpts = parse_command_line(argc, argv, opts_);
    pOpts_.push_back(cmdOpts);
    variables_map vm;
    store(cmdOpts, vm);
    if (vm.count("help")) {
      BOOST_LOG_TRIVIAL(info) << opts_;
      return false;
    }
    const parsed_options envOpts =
        parse_environment(opts_, boost::function1<string, string>([&](string envKey) {
                            transform(envKey.begin(), envKey.end(), envKey.begin(), ::toupper);
                            return opts_.find_nothrow(envKey, false, false, false) ? envKey : "";
                          }));
    pOpts_.push_back(envOpts);
    store(envOpts, vm);

    parseConfigs(vm);

    variables_map vm_final;
    for_each(pOpts_.begin(), pOpts_.end(), [&vm_final](auto& pOpts) { store(pOpts, vm_final); });

    for (Parameter* const param : params_) {
      for (auto& [p1, p2] : param->getConflicts()) {
        conflicting_options(vm_final, p1, p2);
      }
      for (auto& [p1, p2] : param->getDependencies()) {
        option_dependency(vm_final, p1, p2);
      }
    }

    notify(vm_final);
    for (Parameter* param : params_) {
      param->notify();
    }
    BOOST_LOG_TRIVIAL(info) << param_;
    return true;
  } catch (exception& e) {
    BOOST_LOG_TRIVIAL(info) << e.what();
    BOOST_LOG_TRIVIAL(error) << opts_;
    BOOST_THROW_EXCEPTION(e);
  }
}

void ParameterParser::parseConfigs(const variables_map& vmArg) {  // NOLINT(misc-no-recursion)
  vector<string> configs;
  if (vmArg.count(ParserParameter::getConfigsOpt())) {
    auto cs = vmArg[ParserParameter::getConfigsOpt()].as<vector<string>>();
    configs.insert(configs.end(), cs.begin(), cs.end());
  }
  if (vmArg.count(ParserParameter::getConfigConfigsOpt())) {
    auto cs = vmArg[ParserParameter::getConfigConfigsOpt()].as<vector<string>>();
    configs.insert(configs.end(), cs.begin(), cs.end());
  }
  // latest config have higher priority
  for (auto it = configs.rbegin(); it != configs.rend(); ++it) {
    const string& config = *it;
    // check if config parsed then return
    if (pCfgs_.find(config) != pCfgs_.end()) {
      return;
    }
    ifstream ifs(config.c_str());
    if (!ifs.good()) {
      BOOST_THROW_EXCEPTION(runtime_error("'" + config + "' not found"));
    }
    const parsed_options cfgOpts = parse_config_file(ifs, opts_, true);
    variables_map        vm;
    store(cfgOpts, vm);
    pOpts_.push_back(cfgOpts);
    pCfgs_.insert(config);

    parseConfigs(vm);
  }
}

void ParameterParser::conflicting_options(const variables_map& vm, const string& opt1, const string& opt2) {
  if (vm.count(opt1) && !vm[opt1].defaulted() && vm.count(opt2) && !vm[opt2].defaulted())
    BOOST_THROW_EXCEPTION(logic_error(string("Conflicting options '") + opt1 + "' and '" + opt2 + "'."));
}

void ParameterParser::option_dependency(const variables_map& vm, const string& opt, const string& requiredOpt) {
  if (vm.count(opt) && !vm[opt].defaulted())
    if (vm.count(requiredOpt) == 0 || vm[requiredOpt].defaulted())
      BOOST_THROW_EXCEPTION(logic_error(string("Option '") + opt + "' requires option '" + requiredOpt + "'."));
}

}  // namespace jpcc
