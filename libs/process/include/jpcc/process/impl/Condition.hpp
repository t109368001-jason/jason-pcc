#pragma once

namespace jpcc::process {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool Condition::predict(const PointT& point) const {
  double val;
  switch (field) {
    case Condition::X: val = point.x; break;
    case Condition::Y: val = point.y; break;
    case Condition::Z: val = point.z; break;
    case Condition::R: val = point.getVector3fMap().norm(); break;
    default: return false;
  }

  return predictValue(val);
}

}  // namespace jpcc::process