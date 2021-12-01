#include <jpcc/common/Frame.h>

#include <jpcc/common/Transform.h>

namespace jpcc {
namespace common {

void Frame::resize(const size_t size) {
  if (hasAzimuth()) { azimuths_.resize(size); }
  if (hasVertical()) { verticals_.resize(size); }
  if (hasDistance()) { distances_.resize(size); }
  if (hasIntensity()) { intensities_.resize(size); }
  if (hasId()) { ids_.resize(size); }
  if (hasTime()) { times_.resize(size); }
  if (hasPoint()) { points_.resize(size); }
}

void Frame::add(const Laser& laser) {
  size_t index = size();
  resize(index + 1);
  if (hasAzimuth()) { azimuths_[index] = laser.azimuth; }
  if (hasVertical()) { verticals_[index] = laser.vertical; }
  if (hasDistance()) { distances_[index] = laser.distance; }
  if (hasIntensity()) { intensities_[index] = laser.intensity; }
  if (hasId()) { ids_[index] = laser.id; }
  if (hasTime()) { times_[index] = laser.time; }
  if (hasPoint()) { points_[index] = laser2Point(laser); }
}

size_t Frame::size() const {
  if (hasAzimuth()) { return azimuths_.size(); }
  if (hasVertical()) { return verticals_.size(); }
  if (hasDistance()) { return distances_.size(); }
  if (hasIntensity()) { return intensities_.size(); }
  if (hasId()) { return ids_.size(); }
  if (hasTime()) { return times_.size(); }
  if (hasPoint()) { return points_.size(); }
  return 0;
}

std::ostream& operator<<(std::ostream& out, Frame& obj) {
  out << "Frame(";
  out << "azimuths=" << obj.getAzimuths().size() << ", ";
  out << "verticals=" << obj.getVerticals().size() << ", ";
  out << "distances=" << obj.getDistances().size() << ", ";
  out << "intensities=" << obj.getIntensities().size() << ", ";
  out << "ids=" << obj.getIds().size() << ", ";
  out << "times=" << obj.getTimes().size() << ", ";
  out << "points=" << obj.getPoints().size() << ")";
  return out;
}

}  // namespace common
}  // namespace jpcc
