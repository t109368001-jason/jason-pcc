#include <jpcc/process/Condition.h>

#include <algorithm>
#include <stdexcept>

#include <boost/algorithm/string.hpp>

namespace jpcc::process {

using namespace std;
using namespace Eigen;

//////////////////////////////////////////////////////////////////////////////////////////////
Condition::Condition() : type(), operation(), threshold() {}

//////////////////////////////////////////////////////////////////////////////////////////////
Condition::Condition(const string& condition) {  // NOLINT(misc-no-recursion)
  vector<string> ss;
  if (boost::icontains(condition, "&&") || boost::icontains(condition, "||")) {
    if (boost::icontains(condition, "&&") && boost::icontains(condition, "||")) {
      BOOST_THROW_EXCEPTION(logic_error("\"&&\" and \"||\" cannot be use at the same time"));
    } else if (boost::icontains(condition, "&&")) {
      this->type = AND;
    } else {
      this->type = OR;
    }
    vector<string> cs;
    boost::algorithm::split(cs, condition, boost::is_any_of("&&"));
    cs.erase(remove_if(cs.begin(), cs.end(), [](const string& s) { return s.empty(); }), cs.end());
    transform(cs.begin(), cs.end(), back_inserter(this->conditions),  //
              [](auto& c) { return Condition(c); }                    // NOLINT(misc-no-recursion)
    );
    return;
  } else if (!boost::icontains(condition, " ")) {
    BOOST_THROW_EXCEPTION(
        logic_error("please add space before and after operator, (e.g. x > 10, [1.0,0.0,0.0]*p > 10)"));
  }
  boost::algorithm::split(ss, condition, boost::is_any_of(" "));
  ss.erase(remove_if(ss.begin(), ss.end(), [](const string& s) { return s.empty(); }), ss.end());
  assert(ss.size() == 3);

  const string f = boost::to_lower_copy(ss.front());

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
    this->coefficient = make_shared<Vector4f>(Vector4f::Zero());
    vector<string> vector;
    boost::algorithm::split(vector, f, boost::is_any_of(",[]*p"));
    vector.erase(remove_if(vector.begin(), vector.end(), [](const string& s) { return s.empty(); }), vector.end());
    assert(vector.size() == 3 || vector.size() == 4);
    for (int i = 0; i < vector.size(); i++) { (*this->coefficient)(i) = stof(vector[i]); }
  } else {
    BOOST_THROW_EXCEPTION(logic_error("not support type: " + f));
  }

  const string& o = ss[1];
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
    BOOST_THROW_EXCEPTION(logic_error("not support type: " + o));
  }
  this->threshold = stod(ss[2]);
}

Condition::Condition(const ConditionType& type, const vector<string>& conditions) : operation(NONE), threshold() {
  this->type = type;
  transform(conditions.begin(), conditions.end(), back_inserter(this->conditions),
            [](auto& c) { return Condition(c); });
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool Condition::predict(const PointType& point) const {  // NOLINT(misc-no-recursion)
  switch (type) {
    case X:
    case Y:
    case Z:
    case R:
    case PROD: return predictVector3fMap(point);
    case AND:
      return all_of(conditions.begin(), conditions.end(),
                    [&point](const auto& condition) { return condition.predict(point); }  // NOLINT(misc-no-recursion)
      );
    case OR:
      return any_of(conditions.begin(), conditions.end(),
                    [&point](const auto& condition) { return condition.predict(point); }  // NOLINT(misc-no-recursion)
      );
    default: return false;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool Condition::predictVector3fMap(const PointType& point) const {
  double val;
  switch (type) {
    case X: val = point.x(); break;
    case Y: val = point.y(); break;
    case Z: val = point.z(); break;
    case R: val = point.getNorm2<double>(); break;
    case PROD: {
      Vector4f v(float(point.x()), float(point.y()), float(point.z()), 1);
      val = coefficient->transpose() * v;
      break;
    }
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