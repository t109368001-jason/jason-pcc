#include <jpcc/common/Parameter.h>

#include <utility>

namespace jpcc::common {

using namespace std;
using namespace po;

Parameter::Parameter(string prefix, const string& caption) :
    prefix_(std::move(prefix)), caption_(caption), opts_(caption) {}

options_description& Parameter::getOpts() { return opts_; }

vector<array<string, 2>> Parameter::getConflicts() const { return {}; }

vector<array<string, 2>> Parameter::getDependencies() const { return {}; }

void Parameter::notify() {}

}  // namespace jpcc::common
