#include <jpcc/common/Parameter.h>

#include <utility>

namespace jpcc {

using namespace std;
using namespace po;

Parameter::Parameter(string prefix, const string& caption) : prefix_(move(prefix)), caption_(caption), opts_(caption) {}

const options_description& Parameter::getOpts() const { return opts_; }

vector<array<string, 2>> Parameter::getConflicts() const { return {}; }

vector<array<string, 2>> Parameter::getDependencies() const { return {}; }

void Parameter::notify() {}

}  // namespace jpcc
