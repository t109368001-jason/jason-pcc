#include <jpcc/common/ParserParameter.h>

namespace jpcc {
namespace common {

namespace po = boost::program_options;

ParserParameter::ParserParameter() : Parameter("ParserOptions"), disableEnvVar(false) {
  opts_.add_options()                                                                        //
      ("disableEnvVar",                                                                      //
       po::value<bool>(&disableEnvVar)->default_value(disableEnvVar),                        //
       "disable parse environment variable")                                                 //
      ("config", po::value<std::vector<std::string>>(&config)->composing(), "config files")  //
      ;
  ;
}

std::ostream& operator<<(std::ostream& out, const ParserParameter& obj) {
  out << "ParserParameter (command line only)" << std::endl;
  out << "\tdisableEnvVar=" << (obj.disableEnvVar ? "true" : "false") << std::endl;
  out << "\tconfig=";
  for (size_t i = 0; i < obj.config.size(); i++) {
    if (i == 0) { out << "["; }
    if (i != 0) { out << ", "; }
    out << obj.config[i];
    if (i == (obj.config.size() - 1)) { out << "]" << std::endl; }
  }
  return out;
}

}  // namespace common
}  // namespace jpcc
