#pragma once

#include <pcl/point_cloud.h>

#include <PCCPointSet.h>

#include <jpcc/common/Types.h>

namespace jpcc {

class JPCCPointSet3 : public pcc::PCCPointSet3 {
 public:
  using Ptr        = std::shared_ptr<JPCCPointSet3>;
  using ConstPtr   = std::shared_ptr<const JPCCPointSet3>;
  using NormalType = pcc::Vec3<float>;

 protected:
  Index                   frameNumber;
  std::vector<NormalType> normals;
  bool                    withNormals;

 public:
  JPCCPointSet3() : pcc::PCCPointSet3() { withNormals = false; }

  JPCCPointSet3(const PCCPointSet3& src) : pcc::PCCPointSet3(src) { withNormals = false; }

  void swap(JPCCPointSet3& other) {
    using std::swap;
    pcc::PCCPointSet3::swap(other);
    swap(normals, other.normals);
  }

  [[nodiscard]] Index getFrameNumber() const { return frameNumber; }

  Index& getFrameNumber() { return frameNumber; }

  [[nodiscard]] const NormalType& getNormal(const size_t index) const {
    assert(index < normals.size() && withNormals);
    return normals[index];
  }
  NormalType& getNormal(const size_t index) {
    assert(index < normals.size() && withNormals);
    return normals[index];
  }
  void setNormal(const size_t index, const NormalType& normal) {
    assert(index < normals.size() && withNormals);
    normals[index] = normal;
  }
  [[nodiscard]] bool hasNormal() const { return withNormals; }
  void               addNormal() {
    withNormals = true;
    resize(getPointCount());
  }
  void removeNormal() {
    withNormals = false;
    normals.resize(0);
  }

  void resize(const size_t size) override {
    pcc::PCCPointSet3::resize(size);
    if (hasNormal()) { normals.resize(size); }
  }

  void reserve(const size_t size) override {
    pcc::PCCPointSet3::reserve(size);
    if (hasNormal()) { normals.reserve(size); }
  }
  void clear() override {
    pcc::PCCPointSet3::clear();
    normals.clear();
  }

  void append(const JPCCPointSet3& src) {
    if (!getPointCount()) addRemoveAttributes(src);
    int dstEnd = int(this->getPointCount());
    pcc::PCCPointSet3::append(src);
    if (hasNormal() && src.hasNormal())
      std::copy(src.normals.begin(), src.normals.end(), std::next(normals.begin(), dstEnd));
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
  PclFramePtr<PointT> toPcl() const;

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
