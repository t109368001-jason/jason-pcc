#ifndef JPCC_COMMON_FRAME_H_
#define JPCC_COMMON_FRAME_H_

#include <set>

#include <jpcc/common/Common.h>

namespace jpcc {
namespace common {

class Frame {
 public:
  using Ptr = shared_ptr<Frame>;

 protected:
  bool                  hasAzimuth_;
  bool                  hasVertical_;
  bool                  hasDistance_;
  bool                  hasIntensity_;
  bool                  hasId_;
  bool                  hasTime_;
  bool                  hasPoint_;
  std::vector<float>    azimuths_;
  std::vector<float>    verticals_;
  std::vector<float>    distances_;
  std::vector<uint8_t>  intensities_;
  std::vector<uint8_t>  ids_;
  std::vector<uint64_t> times_;
  std::vector<Point>    points_;

 public:
  void addPointTypes(const std::set<std::string>& pointTypes);

  void add(float    azimuth,
           float    vertical,
           float    distance,
           uint8_t  intensity,
           uint8_t  id,
           uint64_t time,
           float    x,
           float    y,
           float    z);

  void shrink_to_fit();

  void reserve(const size_t size);

  void resize(const size_t size);

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

  std::vector<float>& getAzimuths();

  std::vector<float>& getVerticals();

  std::vector<float>& getDistances();

  std::vector<unsigned char>& getIntensities();

  std::vector<unsigned char>& getIds();

  std::vector<uint64_t>& getTimes();

  std::vector<Point>& getPoints();

  friend std::ostream& operator<<(std::ostream& out, Frame& obj);
};

}  // namespace common
}  // namespace jpcc

#endif  // JPCC_COMMON_FRAME_H_