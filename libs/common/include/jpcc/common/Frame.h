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
  bool                 loaded_;
  bool                 hasAzimuth_;
  bool                 hasVertical_;
  bool                 hasDistance_;
  bool                 hasIntensity_;
  bool                 hasId_;
  bool                 hasTime_;
  bool                 hasPoint_;
  int64_t              timestamp_;  // ms
  std::vector<float>   azimuths_;
  std::vector<float>   verticals_;
  std::vector<float>   distances_;
  std::vector<uint8_t> intensities_;
  std::vector<uint8_t> ids_;
  std::vector<int64_t> times_;
  std::vector<Point>   points_;

 public:
  Frame();

  void addPointTypes(const std::set<std::string>& pointTypes);

  void add(float x, float y, float z);

  void add(float   x,
           float   y,
           float   z,
           uint8_t intensity,
           float   azimuth,
           float   vertical,
           float   distance,
           uint8_t id,
           int64_t time);

  void setLoaded(bool loaded = true);

  void setTimestamp(int64_t timestamp);

  void shrink_to_fit();

  void reserve(const size_t size);

  void resize(const size_t size);

  size_t size() const;

  void addPoints();

  void addAzimuths();

  void addVerticals();

  void addDistances();

  void addIntensities();

  void addIds();

  void addTimes();

  bool isLoaded() const;

  bool hasPoint() const;

  bool hasAzimuth() const;

  bool hasVertical() const;

  bool hasDistance() const;

  bool hasIntensity() const;

  bool hasId() const;

  bool hasTime() const;

  int64_t getTimestamp();

  std::vector<Point>& getPoints();

  std::vector<float>& getAzimuths();

  std::vector<float>& getVerticals();

  std::vector<float>& getDistances();

  std::vector<unsigned char>& getIntensities();

  std::vector<unsigned char>& getIds();

  std::vector<int64_t>& getTimes();

  friend std::ostream& operator<<(std::ostream& out, Frame& obj);
};

}  // namespace common
}  // namespace jpcc

#endif  // JPCC_COMMON_FRAME_H_