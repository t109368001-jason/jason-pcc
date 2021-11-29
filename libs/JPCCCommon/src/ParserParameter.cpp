#include <jpcc/common/ParserParameter.h>

namespace jpcc {
namespace common {

using namespace std;
using namespace po;

#define DISABLE_ENV_VAR_OPT "disableEnvVar"
#define CONFIG_OPT "config"

ParserParameter::ParserParameter() : disableEnvVar(false), config() {}

options_description ParserParameter::getOpts() {
  options_description opts("ParserOptions");
  opts.add_options()                                                             //
      (DISABLE_ENV_VAR_OPT,                                                      //
       value<bool>(&disableEnvVar)->default_value(disableEnvVar),                //
       "disable parse environment variable")                                     //
      (CONFIG_OPT, value<vector<string>>(&config)->composing(), "config files")  //
      ;
  return opts;
}

ostream& operator<<(ostream& out, const ParserParameter& obj) {
  out << "ParserParameter (command line only)" << endl;
  out << "\t" DISABLE_ENV_VAR_OPT "=" << (obj.disableEnvVar ? "true" : "false") << endl;
  out << "\t" CONFIG_OPT "=";
  for (size_t i = 0; i < obj.config.size(); i++) {
    if (i == 0) { out << "["; }
    if (i != 0) { out << ", "; }
    out << obj.config[i];
    if (i == (obj.config.size() - 1)) { out << "]" << endl; }
  }
  return out;
}

}  // namespace common
}  // namespace jpcc
