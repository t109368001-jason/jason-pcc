#ifndef JPCC_COMMON_FRAME_H_
#define JPCC_COMMON_FRAME_H_

#include <jpcc/common/Common.h>

namespace jpcc {
namespace common {

class Frame {
 protected:
  bool                       hasAzimuth_;
  bool                       hasVertical_;
  bool                       hasDistance_;
  bool                       hasIntensity_;
  bool                       hasId_;
  bool                       hasTime_;
  bool                       hasPoint_;
  std::vector<double>        azimuths_;
  std::vector<double>        verticals_;
  std::vector<float>         distances_;
  std::vector<unsigned char> intensities_;
  std::vector<unsigned char> ids_;
  std::vector<long long>     times_;
  std::vector<Point>         points_;

 public:
  void addAzimuths() { hasAzimuth_ = true; }

  void addVerticals() { hasVertical_ = true; }

  void addDistances() { hasDistance_ = true; }

  void addIntensities() { hasIntensity_ = true; }

  void addIds() { hasId_ = true; }

  void addTimes() { hasTime_ = true; }

  void addPoints() { hasPoint_ = true; }

  bool hasAzimuth() const { return hasAzimuth_; }

  bool hasVertical() const { return hasVertical_; }

  bool hasDistance() const { return hasDistance_; }

  bool hasIntensity() const { return hasIntensity_; }

  bool hasId() const { return hasId_; }

  bool hasTime() const { return hasTime_; }

  bool hasPoint() const { return hasPoint_; }

  std::vector<double>& getAzimuths() { return azimuths_; };

  std::vector<double>& getVerticals() { return verticals_; };

  std::vector<float>& getDistances() { return distances_; };

  std::vector<unsigned char>& getIntensities() { return intensities_; };

  std::vector<unsigned char>& getIds() { return ids_; };

  std::vector<long long>& getTimes() { return times_; };

  std::vector<Point>& getPoints() { return points_; };

  void resize(const size_t size);

  void add(const Laser& laser);

  size_t size() const;

  friend std::ostream& operator<<(std::ostream& out, Frame& obj);
};

}  // namespace common
}  // namespace jpcc

#endif  // JPCC_COMMON_FRAME_H_