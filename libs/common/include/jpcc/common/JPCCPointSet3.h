#pragma once

#include <pcl/point_cloud.h>

#include <PCCPointSet.h>

#include <jpcc/common/Types.h>

namespace jpcc {

using PointValueType = int32_t;
using PointType      = pcc::PCCPointSet3::PointType;
using NormalType     = pcc::Vec3<float>;

class JPCCPointSet3 : public pcc::PCCPointSet3 {
 public:
  using Ptr      = std::shared_ptr<JPCCPointSet3>;
  using ConstPtr = std::shared_ptr<const JPCCPointSet3>;

 protected:
  Index                   frameNumber_;
  std::vector<NormalType> normals_;
  bool                    withNormals_;

 public:
  JPCCPointSet3() : pcc::PCCPointSet3(), frameNumber_(0), normals_(), withNormals_(false) {}

  JPCCPointSet3(const PCCPointSet3& src)  // NOLINT(google-explicit-constructor)
      :
      pcc::PCCPointSet3(src), frameNumber_(0), normals_(), withNormals_(false) {}

  JPCCPointSet3& operator=(const JPCCPointSet3& rhs) = default;

  ~JPCCPointSet3() = default;

  void swap(JPCCPointSet3& other) {
    using std::swap;
    pcc::PCCPointSet3::swap(other);
    swap(normals_, other.normals_);
  }

  [[nodiscard]] Index getFrameNumber() const { return frameNumber_; }

  [[nodiscard]] Index& getFrameNumber() { return frameNumber_; }

  void setFrameNumber(const Index frameNumber) { this->frameNumber_ = frameNumber; }

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
  [[nodiscard]] bool hasNormal() const { return withNormals_; }
  void               addNormal() {
    withNormals_ = true;
    resize(getPointCount());
  }
  void removeNormal() {
    withNormals_ = false;
    normals_.resize(0);
  }

  void resize(const size_t size) override {
    pcc::PCCPointSet3::resize(size);
    if (hasNormal()) { normals_.resize(size); }
  }

  void reserve(const size_t size) override {
    pcc::PCCPointSet3::reserve(size);
    if (hasNormal()) { normals_.reserve(size); }
  }
  void clear() override {
    pcc::PCCPointSet3::clear();
    normals_.clear();
  }

  void append(const JPCCPointSet3& src) {
    if (!getPointCount()) addRemoveAttributes(src);
    int dstEnd = int(this->getPointCount());
    pcc::PCCPointSet3::append(src);
    if (hasNormal() && src.hasNormal())
      std::copy(src.normals_.begin(), src.normals_.end(), std::next(normals_.begin(), dstEnd));
  }

  void swapPoints(const size_t index1, const size_t index2) override {
    pcc::PCCPointSet3::swapPoints(index1, index2);
    if (hasNormal()) { std::swap(getNormal(index1), getNormal(index2)); }
  }

  void addRemoveAttributes(const JPCCPointSet3& ref) {
    pcc::PCCPointSet3::addRemoveAttributes(ref);
    ref.hasNormal() ? addNormal() : removeNormal();
  }

  void addPoint(const PointType& point);

  void addPositionNormal(const PointType& point, const NormalType& normal);

  void subset(JPCCPointSet3& frame, const Indices& indices);

  template <typename PointT>
  void toPcl(const PclFramePtr<PointT>& pclFrame) const;

  template <typename PointT>
  [[nodiscard]] PclFramePtr<PointT> toPcl() const;

  template <typename PointT>
  void fromPcl(const PclFramePtr<PointT>& pclFrame);
};

using Frame         = JPCCPointSet3;
using FramePtr      = Frame::Ptr;
using FrameConstPtr = Frame::ConstPtr;

using GroupOfFrame    = std::vector<FramePtr>;
using GroupOfFrameMap = std::map<std::string, GroupOfFrame>;

}  // namespace jpcc

#include <jpcc/common/impl/JPCCPointSet3.hpp>
