#pragma once

#include <PCCPointSet.h>

namespace jpcc {

class JPCCPointSet3 : public pcc::PCCPointSet3 {
 public:
  using NormalType = pcc::Vec3<double>;

 protected:
  std::vector<NormalType> normals;
  bool                    withNormals;

 public:
  JPCCPointSet3() : pcc::PCCPointSet3() { withNormals = false; }

  void swap(JPCCPointSet3& other) {
    using std::swap;
    pcc::PCCPointSet3::swap(other);
    swap(normals, other.normals);
  }

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
    pcc::PCCPointSet3::append(src);
    int dstEnd = int(this->getPointCount());
    if (hasColors() && src.hasColors())
      std::copy(src.normals.begin(), src.normals.end(), std::next(normals.begin(), dstEnd));
  }

  void swapPoints(const size_t index1, const size_t index2) override {
    pcc::PCCPointSet3::swapPoints(index1, index2);
    if (hasNormal()) { std::swap(getNormal(index1), getNormal(index2)); }
  }
};

}  // namespace jpcc
