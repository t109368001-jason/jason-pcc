#include <jpcc/common/Parameter.h>

#include <exception>

namespace jpcc {
namespace common {

using namespace std;
using namespace po;

options_description Parameter::getOpts() { throw runtime_error("Parameter: getOpts not implemented"); };

vector<array<string, 2>> Parameter::getConflicts() { return {}; }

vector<array<string, 2>> Parameter::getDependencies() { return {}; }

void Parameter::check() const {}

}  // namespace common
}  // namespace jpcc
