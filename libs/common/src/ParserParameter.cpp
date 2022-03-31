#include <jpcc/common/ParserParameter.h>

namespace jpcc {

using namespace std;
using namespace po;

#define DISABLE_ENV_VAR_OPT "disableEnvVar"
#define CONFIG_OPT "configs"

ParserParameter::ParserParameter() : ParserParameter("", "ParserParameter") {}

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
  out << obj.caption_ << " (command line only)" << endl;
  out << "\t" << obj.prefix_ << DISABLE_ENV_VAR_OPT "=" << (obj.disableEnvVar ? "true" : "false") << endl;
  out << "\t" << obj.prefix_ << CONFIG_OPT "=";
  for (size_t i = 0; i < obj.configs.size(); i++) {
    if (i == 0) { out << "["; }
    if (i != 0) { out << ", "; }
    out << obj.configs.at(i);
    if (i == (obj.configs.size() - 1)) { out << "]"; }
  }
  out << endl;
  return out;
}

}  // namespace jpcc
