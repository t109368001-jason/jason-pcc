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
  void resize(const size_t size);

  void add(const Laser& laser);

  size_t size() const;

  void addAzimuths();

  void addVerticals();

  void addDistances();

  void addIntensities();

  void addIds();

  void addTimes();

  void addPoints();

  bool hasAzimuth() const;

  bool hasVertical() const;

  bool hasDistance() const;

  bool hasIntensity() const;

  bool hasId() const;

  bool hasTime() const;

  bool hasPoint() const;

  std::vector<double>& getAzimuths();

  std::vector<double>& getVerticals();

  std::vector<float>& getDistances();

  std::vector<unsigned char>& getIntensities();

  std::vector<unsigned char>& getIds();

  std::vector<long long>& getTimes();

  std::vector<Point>& getPoints();

  friend std::ostream& operator<<(std::ostream& out, Frame& obj);
};

}  // namespace common
}  // namespace jpcc

#endif  // JPCC_COMMON_FRAME_H_