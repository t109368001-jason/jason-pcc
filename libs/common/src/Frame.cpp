#include <jpcc/common/Frame.h>

namespace jpcc::common {

using namespace std;

Frame::Frame() :
    loaded_(false),
    hasAzimuth_(false),
    hasVertical_(false),
    hasDistance_(false),
    hasIntensity_(false),
    hasId_(false),
    hasTime_(false),
    hasPoint_(false),
    timestamp_(0),
    azimuths_(),
    verticals_(),
    distances_(),
    intensities_(),
    ids_(),
    times_(),
    points_() {}

void Frame::addPointTypes(const set<string>& pointTypes) {
  if (pointTypes.find("xyz") != pointTypes.end()) { addPoints(); }
  if (pointTypes.find("azimuth") != pointTypes.end()) { addAzimuths(); }
  if (pointTypes.find("vertical") != pointTypes.end()) { addVerticals(); }
  if (pointTypes.find("distance") != pointTypes.end()) { addDistances(); }
  if (pointTypes.find("intensity") != pointTypes.end()) { addIntensities(); }
  if (pointTypes.find("id") != pointTypes.end()) { addIds(); }
  if (pointTypes.find("time") != pointTypes.end()) { addTimes(); }
}

void Frame::add(float x, float y, float z) {
  assert(!loaded_);
  assert(hasPoint());
  points_.emplace_back(x, y, z);
}

void Frame::add(float   x,
                float   y,
                float   z,
                uint8_t intensity,
                float   azimuth,
                float   vertical,
                float   distance,
                uint8_t id,
                int64_t time) {
  assert(!loaded_);
  assert(hasAzimuth());
  assert(hasVertical());
  assert(hasDistance());
  assert(hasIntensity());
  assert(hasId());
  assert(hasTime());
  assert(hasPoint());
  { azimuths_.push_back(azimuth); }
  { verticals_.push_back(vertical); }
  { distances_.push_back(distance); }
  { intensities_.push_back(intensity); }
  { ids_.push_back(id); }
  { times_.push_back(time); }
  { points_.emplace_back(x, y, z); }
}

void Frame::setLoaded(bool loaded) {
  loaded_ = loaded;
  if (loaded) { shrink_to_fit(); }
}

void Frame::setTimestamp(int64_t timestamp) { timestamp_ = timestamp; }

void Frame::shrink_to_fit() {
  if (hasPoint()) { points_.shrink_to_fit(); }
  if (hasAzimuth()) { azimuths_.shrink_to_fit(); }
  if (hasVertical()) { verticals_.shrink_to_fit(); }
  if (hasDistance()) { distances_.shrink_to_fit(); }
  if (hasIntensity()) { intensities_.shrink_to_fit(); }
  if (hasId()) { ids_.shrink_to_fit(); }
  if (hasTime()) { times_.shrink_to_fit(); }
}

void Frame::reserve(const size_t size) {
  if (hasPoint()) { points_.reserve(size); }
  if (hasAzimuth()) { azimuths_.reserve(size); }
  if (hasVertical()) { verticals_.reserve(size); }
  if (hasDistance()) { distances_.reserve(size); }
  if (hasIntensity()) { intensities_.reserve(size); }
  if (hasId()) { ids_.reserve(size); }
  if (hasTime()) { times_.reserve(size); }
}

void Frame::resize(const size_t size) {
  if (this->size() == size) { return; }
  if (hasPoint()) { points_.resize(size); }
  if (hasAzimuth()) { azimuths_.resize(size); }
  if (hasVertical()) { verticals_.resize(size); }
  if (hasDistance()) { distances_.resize(size); }
  if (hasIntensity()) { intensities_.resize(size); }
  if (hasId()) { ids_.resize(size); }
  if (hasTime()) { times_.resize(size); }
  setLoaded(false);
}

size_t Frame::size() const {
  if (hasPoint()) { return points_.size(); }
  if (hasAzimuth()) { return azimuths_.size(); }
  if (hasVertical()) { return verticals_.size(); }
  if (hasDistance()) { return distances_.size(); }
  if (hasIntensity()) { return intensities_.size(); }
  if (hasId()) { return ids_.size(); }
  if (hasTime()) { return times_.size(); }
  return 0;
}

void Frame::addPoints() { hasPoint_ = true; }

void Frame::addAzimuths() { hasAzimuth_ = true; }

void Frame::addVerticals() { hasVertical_ = true; }

void Frame::addDistances() { hasDistance_ = true; }

void Frame::addIntensities() { hasIntensity_ = true; }

void Frame::addIds() { hasId_ = true; }

void Frame::addTimes() { hasTime_ = true; }

bool Frame::isLoaded() const { return loaded_; }

bool Frame::hasPoint() const { return hasPoint_; }

bool Frame::hasAzimuth() const { return hasAzimuth_; }

bool Frame::hasVertical() const { return hasVertical_; }

bool Frame::hasDistance() const { return hasDistance_; }

bool Frame::hasIntensity() const { return hasIntensity_; }

bool Frame::hasId() const { return hasId_; }

bool Frame::hasTime() const { return hasTime_; }

int64_t Frame::getTimestamp() const { return timestamp_; }

vector<Point>& Frame::getPoints() { return points_; }

vector<float>& Frame::getAzimuths() { return azimuths_; }

vector<float>& Frame::getVerticals() { return verticals_; }

vector<float>& Frame::getDistances() { return distances_; }

vector<unsigned char>& Frame::getIntensities() { return intensities_; }

vector<unsigned char>& Frame::getIds() { return ids_; }

vector<int64_t>& Frame::getTimes() { return times_; }

ostream& operator<<(ostream& out, Frame& obj) {
  out << "Frame(";
  bool flag = false;
  if (obj.hasPoint()) {
    out << "points=" << obj.getPoints().size();
    flag = true;
  }
  if (obj.hasAzimuth()) {
    if (flag) {
      out << ", ";
      flag = true;
    }
    out << "azimuths=" << obj.getAzimuths().size();
  }
  if (obj.hasVertical()) {
    if (flag) {
      out << ", ";
      flag = true;
    }
    out << "verticals=" << obj.getVerticals().size();
  }
  if (obj.hasDistance()) {
    if (flag) {
      out << ", ";
      flag = true;
    }
    out << "distances=" << obj.getDistances().size();
  }
  if (obj.hasIntensity()) {
    if (flag) {
      out << ", ";
      flag = true;
    }
    out << "intensities=" << obj.getIntensities().size();
  }
  if (obj.hasId()) {
    if (flag) {
      out << ", ";
      flag = true;
    }
    out << "ids=" << obj.getIds().size();
  }
  if (obj.hasTime()) {
    if (flag) { out << ", "; }
    out << "times=" << obj.getTimes().size();
  }
  out << ")";
  return out;
}

}  // namespace jpcc::common
