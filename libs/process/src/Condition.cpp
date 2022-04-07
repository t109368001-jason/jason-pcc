#include <jpcc/process/Condition.h>

#include <boost/algorithm/string.hpp>

namespace jpcc::process {

using namespace std;

Condition::Condition() : field(), operation(), threshold() {}

Condition::Condition(const std::string& condition) {
  vector<string> ss;
  if (!boost::icontains(condition, " ")) {
    BOOST_THROW_EXCEPTION(logic_error("please split field, operation, threshold by space, (e.g. x > 10)"));
  }
  boost::algorithm::split(ss, condition, boost::is_any_of(" "));
  assert(ss.size() == 3);

  const string f = boost::to_lower_copy(ss.at(0));

  if (f == "r") {
    this->field = R;
  } else if (f == "x") {
    this->field = X;
  } else if (f == "y") {
    this->field = Y;
  } else if (f == "z") {
    this->field = Z;
  } else {
    BOOST_THROW_EXCEPTION(logic_error("not support field: " + f));
  }

  const string& o = ss.at(1);
  if ((o == "=") || (o == "==")) {
    this->operation = EQ;
  } else if (o == ">") {
    this->operation = GT;
  } else if (o == ">=") {
    this->operation = GE;
  } else if (o == "<") {
    this->operation = LT;
  } else if (o == "<=") {
    this->operation = LE;
  } else {
    BOOST_THROW_EXCEPTION(logic_error("not support operation: " + o));
  }
  this->threshold = stod(ss.at(2));
}

}  // namespace jpcc::process