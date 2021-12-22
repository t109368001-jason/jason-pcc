#include <jpcc/common/Parameter.h>

namespace jpcc {
namespace common {

using namespace std;
using namespace po;

Parameter::Parameter(string prefix, string caption) : prefix_(prefix), caption_(caption), opts_(caption) {}

options_description& Parameter::getOpts() { return opts_; };

vector<array<string, 2>> Parameter::getConflicts() { return {}; }

vector<array<string, 2>> Parameter::getDependencies() { return {}; }

void Parameter::notify() {}

}  // namespace common
}  // namespace jpcc
