#include <jpcc/process/Condition.h>

#include <boost/algorithm/string.hpp>

namespace jpcc::process {

using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////
Condition::Condition() : type(), operation(), threshold() {}

//////////////////////////////////////////////////////////////////////////////////////////////
Condition::Condition(const std::string& condition) {
  vector<string> ss;
  if (!boost::icontains(condition, " ")) {
    BOOST_THROW_EXCEPTION(
        logic_error("please add space before and after operator, (e.g. x > 10, [1.0,0.0,0.0]*p > 10)"));
  }
  boost::algorithm::split(ss, condition, boost::is_any_of(" "));
  assert(ss.size() == 3);

  const string f = boost::to_lower_copy(ss.at(0));

  if (f == "r") {
    this->type = R;
  } else if (f == "x") {
    this->type = X;
  } else if (f == "y") {
    this->type = Y;
  } else if (f == "z") {
    this->type = Z;
  } else if (boost::icontains(f, "[") && boost::icontains(f, "]*p")) {
    this->type        = PROD;
    this->coefficient = make_shared<Eigen::Vector4f>(Eigen::Vector4f::Zero());
    vector<string> vector;
    boost::algorithm::split(vector, f, boost::is_any_of(",[]*p"));
    vector.erase(std::remove_if(vector.begin(), vector.end(), [](const string& s) { return s.empty(); }), vector.end());
    assert(vector.size() == 3 || vector.size() == 4);
    for (int i = 0; i < vector.size(); i++) { (*this->coefficient)(i) = stof(vector.at(i)); }
  } else {
    BOOST_THROW_EXCEPTION(logic_error("not support type: " + f));
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

//////////////////////////////////////////////////////////////////////////////////////////////
bool Condition::predictVector3fMap(pcl::Vector3fMapConst& vector3fMap) const {
  double val;
  switch (type) {
    case X: val = vector3fMap(0); break;
    case Y: val = vector3fMap(1); break;
    case Z: val = vector3fMap(2); break;
    case R: val = vector3fMap.norm(); break;
    default: return false;
  }

  return predictValue(val);
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool Condition::predictVector4fMap(pcl::Vector4fMapConst& vector4fMap) const {
  double val;
  switch (type) {
    case PROD: val = coefficient->transpose() * vector4fMap; break;
    default: return false;
  }

  return predictValue(val);
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool Condition::predictValue(const double val) const {
  switch (operation) {
    case GT: return val > threshold;
    case GE: return val >= threshold;
    case LT: return val < threshold;
    case LE: return val <= threshold;
    case EQ: return val == threshold;
    default: return false;
  }
}

}  // namespace jpcc::process