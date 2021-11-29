#include <jpcc/common/Parameter.h>

namespace jpcc {
namespace common {

namespace po = boost::program_options;

Parameter::Parameter(const std::string optsName) : opts_(std::move(optsName)) {}

po::options_description& Parameter::getOpts() { return opts_; };

std::vector<std::array<std::string, 2>> Parameter::getConflicts() { return {}; }

std::vector<std::array<std::string, 2>> Parameter::getDependencies() { return {}; }

}  // namespace common
}  // namespace jpcc
