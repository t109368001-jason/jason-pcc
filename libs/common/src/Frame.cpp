#include <jpcc/common/Frame.h>

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

void Frame::add(float    azimuth,
                float    vertical,
                float    distance,
                uint8_t  intensity,
                uint8_t  id,
                uint64_t time,
                float    x,
                float    y,
                float    z) {
  if (hasAzimuth()) { azimuths_.push_back(azimuth); }
  if (hasVertical()) { verticals_.push_back(vertical); }
  if (hasDistance()) { distances_.push_back(distance); }
  if (hasIntensity()) { intensities_.push_back(intensity); }
  if (hasId()) { ids_.push_back(id); }
  if (hasTime()) { times_.push_back(time); }
  if (hasPoint()) { points_.push_back(Point(x, y, z)); }
}

void Frame::shrink_to_fit() {
  if (hasAzimuth()) { azimuths_.shrink_to_fit(); }
  if (hasVertical()) { verticals_.shrink_to_fit(); }
  if (hasDistance()) { distances_.shrink_to_fit(); }
  if (hasIntensity()) { intensities_.shrink_to_fit(); }
  if (hasId()) { ids_.shrink_to_fit(); }
  if (hasTime()) { times_.shrink_to_fit(); }
  if (hasPoint()) { points_.shrink_to_fit(); }
}

void Frame::reserve(const size_t size) {
  if (hasAzimuth()) { azimuths_.reserve(size); }
  if (hasVertical()) { verticals_.reserve(size); }
  if (hasDistance()) { distances_.reserve(size); }
  if (hasIntensity()) { intensities_.reserve(size); }
  if (hasId()) { ids_.reserve(size); }
  if (hasTime()) { times_.reserve(size); }
  if (hasPoint()) { points_.reserve(size); }
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

vector<float>& Frame::getAzimuths() { return azimuths_; };

vector<float>& Frame::getVerticals() { return verticals_; };

vector<float>& Frame::getDistances() { return distances_; };

vector<unsigned char>& Frame::getIntensities() { return intensities_; };

vector<unsigned char>& Frame::getIds() { return ids_; };

vector<uint64_t>& Frame::getTimes() { return times_; };

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
