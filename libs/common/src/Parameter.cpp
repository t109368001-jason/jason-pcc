#include <jpcc/common/Parameter.h>

#include <utility>

namespace jpcc {

using namespace std;
using namespace po;

Parameter::Parameter(string prefix, const string& caption) :
    prefix_(std::move(prefix)), caption_(caption), opts_(caption) {}

const options_description& Parameter::getOpts() const { return opts_; }

vector<array<string, 2>> Parameter::getConflicts() const { return {}; }

vector<array<string, 2>> Parameter::getDependencies() const { return {}; }

vector<string> Parameter::getShowTexts() const {
  vector<string> showTexts;
  getShowTexts(showTexts);
  return showTexts;
}

void Parameter::getShowTexts(vector<string>& showTexts) const {}

void Parameter::notify() {}

ParameterOStream Parameter::coutParameters(ostream& out) const {
  out << caption_ << endl;
  return {out, prefix_};
}

}  // namespace jpcc
