#pragma once

namespace jpcc::process {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool Condition::predict(const PointT& point) const {
  switch (type) {
    case X:
    case Y:
    case Z:
    case R:
    case PROD: return predictVector3fMap(point.getVector3fMap());
    case REFLECTIVITY: return predictVector4fMap(point.getVector4fMap());
    case AND:
      return std::all_of(conditions.begin(), conditions.end(),
                         [&point](const auto& condition) { return condition.predict(point); });
    case OR:
      return std::any_of(conditions.begin(), conditions.end(),
                         [&point](const auto& condition) { return condition.predict(point); });
    default: return false;
  }
}

}  // namespace jpcc::process