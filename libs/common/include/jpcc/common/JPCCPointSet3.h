#pragma once

#include <jpcc/common/Types.h>

namespace jpcc {

enum JPCCEndianness { JPCC_BIG_ENDIAN = 0, JPCC_LITTLE_ENDIAN = 1 };

template <typename T>
using Vec3 = std::array<T, 3>;

using PointValueType  = int32_t;
using NormalValueType = float;

using PointType  = Vec3<PointValueType>;
using NormalType = Vec3<NormalValueType>;

class JPCCPointSet3 {
 public:
  using Ptr      = std::shared_ptr<JPCCPointSet3>;
  using ConstPtr = std::shared_ptr<const JPCCPointSet3>;

 protected:
  Index                   frameNumber_;
  std::vector<PointType>  positions_;
  std::vector<Intensity>  reflectances_;
  std::vector<NormalType> normals_;
  bool                    withReflectances_;
  bool                    withNormals_;

 public:
  JPCCPointSet3() :
      frameNumber_(0), positions_(), reflectances_(), normals_(), withReflectances_(false), withNormals_(false) {}

  JPCCPointSet3& operator=(const JPCCPointSet3& rhs) = default;

  ~JPCCPointSet3() = default;

  void swap(JPCCPointSet3& other) {
    using std::swap;
    swap(positions_, other.positions_);
    swap(reflectances_, other.reflectances_);
    swap(normals_, other.normals_);
    swap(withReflectances_, other.withReflectances_);
  }

  [[nodiscard]] Index getFrameNumber() const { return frameNumber_; }

  [[nodiscard]] Index& getFrameNumber() { return frameNumber_; }

  void setFrameNumber(const Index frameNumber) { this->frameNumber_ = frameNumber; }

  [[nodiscard]] PointType operator[](const size_t index) const {
    assert(index < positions_.size());
    return positions_[index];
  }
  [[nodiscard]] PointType& operator[](const size_t index) {
    assert(index < positions_.size());
    return positions_[index];
  }

  void swapPoints(std::vector<PointType>& other) { positions_.swap(other); }

  [[nodiscard]] Intensity getReflectance(const size_t index) const {
    assert(index < reflectances_.size() && withReflectances_);
    return reflectances_[index];
  }
  [[nodiscard]] Intensity& getReflectance(const size_t index) {
    assert(index < reflectances_.size() && withReflectances_);
    return reflectances_[index];
  }
  void setReflectance(const size_t index, const Intensity reflectance) {
    assert(index < reflectances_.size() && withReflectances_);
    reflectances_[index] = reflectance;
  }

  [[nodiscard]] bool hasReflectances() const { return withReflectances_; }
  void               addReflectances() {
    withReflectances_ = true;
    resize(getPointCount());
  }
  void removeReflectances() {
    withReflectances_ = false;
    reflectances_.resize(0);
  }

  [[nodiscard]] const NormalType& getNormal(const size_t index) const {
    assert(index < normals_.size() && withNormals_);
    return normals_[index];
  }
  [[nodiscard]] NormalType& getNormal(const size_t index) {
    assert(index < normals_.size() && withNormals_);
    return normals_[index];
  }
  void setNormal(const size_t index, const NormalType& normal) {
    assert(index < normals_.size() && withNormals_);
    normals_[index] = normal;
  }
  [[nodiscard]] bool hasNormals() const { return withNormals_; }
  void               addNormals() {
    withNormals_ = true;
    resize(getPointCount());
  }
  void removeNormals() {
    withNormals_ = false;
    normals_.resize(0);
  }

  [[nodiscard]] size_t getPointCount() const { return positions_.size(); }

  [[nodiscard]] size_t size() const { return positions_.size(); }

  void resize(const size_t size) {
    positions_.resize(size);
    if (hasReflectances()) {
      reflectances_.resize(size);
    }
    if (hasNormals()) {
      normals_.resize(size);
    }
  }

  void reserve(const size_t size) {
    positions_.reserve(size);
    if (hasReflectances()) {
      reflectances_.reserve(size);
    }
    if (hasNormals()) {
      normals_.reserve(size);
    }
  }
  void clear() {
    positions_.clear();
    reflectances_.clear();
    normals_.clear();
  }

  void append(const JPCCPointSet3& src) {
    if (!getPointCount()) addRemoveAttributes(src);

    auto dstEnd  = positions_.size();
    auto srcSize = src.positions_.size();
    resize(dstEnd + srcSize);

    std::copy(src.positions_.begin(), src.positions_.end(), std::next(positions_.begin(), static_cast<int>(dstEnd)));

    if (hasReflectances() && src.hasReflectances())
      std::copy(src.reflectances_.begin(), src.reflectances_.end(),
                std::next(reflectances_.begin(), static_cast<int>(dstEnd)));
    if (hasNormals() && src.hasNormals())
      std::copy(src.normals_.begin(), src.normals_.end(), std::next(normals_.begin(), static_cast<int>(dstEnd)));
  }

  void swapPoints(const size_t index1, const size_t index2) {
    assert(index1 < getPointCount());
    assert(index2 < getPointCount());
    std::swap((*this)[index1], (*this)[index2]);
    if (hasReflectances()) {
      std::swap(getReflectance(index1), getReflectance(index2));
    }
    if (hasNormals()) {
      std::swap(getNormal(index1), getNormal(index2));
    }
  }

  void addRemoveAttributes(const JPCCPointSet3& ref) {
    ref.hasReflectances() ? addReflectances() : removeReflectances();
    ref.hasNormals() ? addNormals() : removeNormals();
  }

  void addPoint(const PointType& point);

  void addPositionNormal(const PointType& point, const NormalType& normal);

  void subset(JPCCPointSet3& frame, const Indices& indices) const;

  template <typename PointT>
  void toPcl(const PclFramePtr<PointT>& pclFrame) const;

  template <typename PointT>
  [[nodiscard]] PclFramePtr<PointT> toPcl() const;

  template <typename PointT>
  void fromPcl(const PclFramePtr<PointT>& pclFrame);

  bool write(const std::string& fileName, const bool asAscii = false);
  bool read(const std::string& fileName, const bool readNormals = false);
};

using Frame         = JPCCPointSet3;
using FramePtr      = Frame::Ptr;
using FrameConstPtr = Frame::ConstPtr;

using GroupOfFrame    = std::vector<FramePtr>;
using GroupOfFrameMap = std::map<std::string, GroupOfFrame>;

}  // namespace jpcc

#include <jpcc/common/impl/JPCCPointSet3.hpp>
