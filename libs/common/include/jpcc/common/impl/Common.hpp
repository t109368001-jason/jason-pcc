#pragma once

#include <iostream>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace jpcc {

std::ostream& operator<<(std::ostream& os, const PointNormal& p);

struct EIGEN_ALIGN16 _Point {
  PCL_ADD_POINT4D;
  PCL_ADD_NORMAL4D;
  union {
    struct {
      float curvature;
    };
    float data_c[4];
  };
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};

struct PointNormal : public _Point {
  inline PointNormal(const _Point& p) {
    x         = p.x;
    y         = p.y;
    z         = p.z;
    data[3]   = 1.0f;
    normal_x  = p.normal_x;
    normal_y  = p.normal_y;
    normal_z  = p.normal_z;
    data_n[3] = 0.0f;
    curvature = p.curvature;
  }

  inline PointNormal() {
    x = y = z = 0.0f;
    data[3]   = 1.0f;
    normal_x = normal_y = normal_z = data_n[3] = 0.0f;
    curvature                                  = 0.f;
  }

  inline PointNormal(const float _x, const float _y, const float _z) {
    x        = _x;
    y        = _y;
    z        = _z;
    data[3]  = 1.0f;
    normal_x = normal_y = normal_z = data_n[3] = 0.0f;
    curvature                                  = 0.f;
  }

  inline PointNormal(
      const float _x, const float _y, const float _z, const float _nx, const float _ny, const float _nz) {
    x         = _x;
    y         = _y;
    z         = _z;
    data[3]   = 1.0f;
    normal_x  = _nx;
    normal_y  = _ny;
    normal_z  = _nz;
    data_n[3] = 0.0f;
    curvature = 0.f;
  }

  friend std::ostream& operator<<(std::ostream& os, const PointNormal& p);
};

}  // namespace jpcc

POINT_CLOUD_REGISTER_POINT_STRUCT(jpcc::PointNormal,
                                  (float, x, x)(float, y, y)(float, z, z)(float, normal_x, normal_x)(
                                      float, normal_y, normal_y)(float, normal_z, normal_z))
