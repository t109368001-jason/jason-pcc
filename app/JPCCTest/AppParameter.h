#pragma once

#include <jpcc/common/Parameter.h>
#include <jpcc/io/DatasetParameter.h>
#include <jpcc/io/DatasetReaderParameter.h>

namespace jpcc {

class AppParameter : public Parameter {
 public:
  bool        parallel;
  bool        bool1;
  bool        bool2;
  bool        bool3;
  int         int1;
  int         int2;
  int         int3;
  float       float1;
  float       float2;
  float       float3;
  std::string string1;
  std::string string2;
  std::string string3;

  AppParameter();
};

}  // namespace jpcc
