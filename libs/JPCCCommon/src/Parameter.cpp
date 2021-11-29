#include <jpcc/common/Parameter.h>

namespace jpcc {
namespace common {

using namespace std;
using namespace po;

Parameter::Parameter(const string optsName) : opts_(move(optsName)) {}

options_description& Parameter::getOpts() { return opts_; };

vector<array<string, 2>> Parameter::getConflicts() { return {}; }

vector<array<string, 2>> Parameter::getDependencies() { return {}; }

}  // namespace common
}  // namespace jpcc
