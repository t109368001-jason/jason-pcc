#include <jpcc/common/ParserParameter.h>

#include <boost/log/trivial.hpp>

namespace jpcc {

using namespace std;
using namespace po;

#define DISABLE_ENV_VAR_OPT "disableEnvVar"
#define CONFIGS_OPT "configs"
#define CONFIG_CONFIGS_OPT "config.configs"

ParserParameter::ParserParameter() : ParserParameter("", "ParserParameter (command line only)") {
}

ParserParameter::ParserParameter(const string& prefix, const string& caption) :
    Parameter(prefix, caption), disableEnvVar(false), configs() {
  opts_.add_options()                                                                                               //
      (string(prefix_ + DISABLE_ENV_VAR_OPT).c_str(),                                                               //
       value<bool>(&disableEnvVar)->default_value(disableEnvVar),                                                   //
       "disable parse environment variable")                                                                        //
      (string(prefix_ + CONFIGS_OPT).c_str(), value<vector<string>>(&configs)->composing(), "config files")         //
      (string(prefix_ + CONFIG_CONFIGS_OPT).c_str(), value<vector<string>>(&configs)->composing(), "config files")  //
      ;
}

string ParserParameter::getConfigsOpt() {
  return CONFIGS_OPT;
}

string ParserParameter::getConfigConfigsOpt() {
  return CONFIG_CONFIGS_OPT;
}

ostream& operator<<(ostream& out, const ParserParameter& obj) {
  obj.coutParameters(out)                       //
      (DISABLE_ENV_VAR_OPT, obj.disableEnvVar)  //
      (CONFIGS_OPT, obj.configs)                //
      ;
  return out;
}

}  // namespace jpcc
