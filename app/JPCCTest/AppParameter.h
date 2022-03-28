#pragma once

#include <array>
#include <iostream>
#include <string>

#include <jpcc/common/Parameter.h>
#include <jpcc/io/DatasetParameter.h>
#include <jpcc/io/DatasetReaderParameter.h>
#include <jpcc/process/PreProcessParameter.h>

namespace jpcc {

class AppParameter : public Parameter {
 protected:
  std::string cameraPosition_;

 public:
  bool                         parallel;
  std::array<double, 9>        cameraPosition;
  io::DatasetParameter         dataset;
  io::DatasetReaderParameter   reader;
  process::PreProcessParameter preProcess;
  bool                         bool1;
  bool                         bool2;
  bool                         bool3;
  int                          int1;
  int                          int2;
  int                          int3;
  float                        float1;
  float                        float2;
  float                        float3;
  std::string                  string1;
  std::string                  string2;
  std::string                  string3;

  AppParameter();

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const AppParameter& obj);

  void applyCameraPosition(const std::function<void(double pos_x,
                                                    double pos_y,
                                                    double pos_z,
                                                    double view_x,
                                                    double view_y,
                                                    double view_z,
                                                    double up_x,
                                                    double up_y,
                                                    double up_z)>& func) const;
};

}  // namespace jpcc
