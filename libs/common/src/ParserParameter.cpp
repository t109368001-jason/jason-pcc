#include <jpcc/common/ParserParameter.h>

namespace jpcc {

using namespace std;
using namespace po;

#define DISABLE_ENV_VAR_OPT "disableEnvVar"
#define CONFIG_OPT "configs"

ParserParameter::ParserParameter() : ParserParameter("", "ParserParameter (command line only)") {}

ParserParameter::ParserParameter(const string& prefix, const string& caption) :
    Parameter(prefix, caption), disableEnvVar(false), configs() {
  opts_.add_options()                                                                                       //
      (string(prefix_ + DISABLE_ENV_VAR_OPT).c_str(),                                                       //
       value<bool>(&disableEnvVar)->default_value(disableEnvVar),                                           //
       "disable parse environment variable")                                                                //
      (string(prefix_ + CONFIG_OPT).c_str(), value<vector<string>>(&configs)->composing(), "config files")  //
      ;
}

string ParserParameter::getConfigsOpt() { return CONFIG_OPT; }

ostream& operator<<(ostream& out, const ParserParameter& obj) {
  obj.coutParameters(out)                       //
      (DISABLE_ENV_VAR_OPT, obj.disableEnvVar)  //
      (CONFIG_OPT, obj.configs)                 //
      ;
  return out;
}

}  // namespace jpcc
