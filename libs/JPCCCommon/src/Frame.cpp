#include <jpcc/common/Frame.h>

#include <jpcc/common/Transform.h>

namespace jpcc {
namespace common {

using namespace std;

void Frame::addPointTypes(const set<string>& pointTypes) {
  if (pointTypes.contains("azimuth")) { addAzimuths(); }
  if (pointTypes.contains("vertical")) { addVerticals(); }
  if (pointTypes.contains("distance")) { addDistances(); }
  if (pointTypes.contains("intensity")) { addIntensities(); }
  if (pointTypes.contains("id")) { addIds(); }
  if (pointTypes.contains("time")) { addTimes(); }
  if (pointTypes.contains("point")) { addPoints(); }
}

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

void Frame::addAzimuths() { hasAzimuth_ = true; }

void Frame::addVerticals() { hasVertical_ = true; }

void Frame::addDistances() { hasDistance_ = true; }

void Frame::addIntensities() { hasIntensity_ = true; }

void Frame::addIds() { hasId_ = true; }

void Frame::addTimes() { hasTime_ = true; }

void Frame::addPoints() { hasPoint_ = true; }

bool Frame::hasAzimuth() const { return hasAzimuth_; }

bool Frame::hasVertical() const { return hasVertical_; }

bool Frame::hasDistance() const { return hasDistance_; }

bool Frame::hasIntensity() const { return hasIntensity_; }

bool Frame::hasId() const { return hasId_; }

bool Frame::hasTime() const { return hasTime_; }

bool Frame::hasPoint() const { return hasPoint_; }

vector<double>& Frame::getAzimuths() { return azimuths_; };

vector<double>& Frame::getVerticals() { return verticals_; };

vector<float>& Frame::getDistances() { return distances_; };

vector<unsigned char>& Frame::getIntensities() { return intensities_; };

vector<unsigned char>& Frame::getIds() { return ids_; };

vector<long long>& Frame::getTimes() { return times_; };

vector<Point>& Frame::getPoints() { return points_; };

ostream& operator<<(ostream& out, Frame& obj) {
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
