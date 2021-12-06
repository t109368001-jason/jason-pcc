#include <jpcc/common/Parameter.h>

#include <exception>

namespace jpcc {
namespace common {

using namespace std;
using namespace po;

options_description Parameter::getOpts() { throw logic_error(string("Not Implemented ") + BOOST_CURRENT_FUNCTION); };

vector<array<string, 2>> Parameter::getConflicts() { return {}; }

vector<array<string, 2>> Parameter::getDependencies() { return {}; }

void Parameter::notify() {}

}  // namespace common
}  // namespace jpcc
