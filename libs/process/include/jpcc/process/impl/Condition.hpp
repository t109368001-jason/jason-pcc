#pragma once

#include <pcl/impl/point_types.hpp>
namespace jpcc::process {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool Condition::predict(const PointT& point) const {
  switch (type) {
    case X:
    case Y:
    case Z:
    case R: return predictVector3fMap(point.getVector3fMap());
    case PROD: return predictVector4fMap(point.getVector4fMap());
    default: return false;
  }
}

}  // namespace jpcc::process