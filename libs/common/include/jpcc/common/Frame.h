#ifndef JPCC_COMMON_FRAME_H_
#define JPCC_COMMON_FRAME_H_

#include <set>

#include <jpcc/common/Common.h>

namespace jpcc::common {

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

  void reserve(size_t size);

  void resize(size_t size);

  [[nodiscard]] size_t size() const;

  void addPoints();

  void addAzimuths();

  void addVerticals();

  void addDistances();

  void addIntensities();

  void addIds();

  void addTimes();

  [[nodiscard]] bool isLoaded() const;

  [[nodiscard]] bool hasPoint() const;

  [[nodiscard]] bool hasAzimuth() const;

  [[nodiscard]] bool hasVertical() const;

  [[nodiscard]] bool hasDistance() const;

  [[nodiscard]] bool hasIntensity() const;

  [[nodiscard]] bool hasId() const;

  [[nodiscard]] bool hasTime() const;

  [[nodiscard]] int64_t getTimestamp() const;

  [[nodiscard]] std::vector<Point>& getPoints();

  [[nodiscard]] std::vector<float>& getAzimuths();

  [[nodiscard]] std::vector<float>& getVerticals();

  [[nodiscard]] std::vector<float>& getDistances();

  [[nodiscard]] std::vector<unsigned char>& getIntensities();

  [[nodiscard]] std::vector<unsigned char>& getIds();

  [[nodiscard]] std::vector<int64_t>& getTimes();

  friend std::ostream& operator<<(std::ostream& out, Frame& obj);
};

}  // namespace jpcc::common

#endif  // JPCC_COMMON_FRAME_H_