#pragma once

#include <iostream>
#include <string>

#include <jpcc/common/Parameter.h>
#include <jpcc/encoder/PCCEncoderTMC2Parameters.h>

namespace jpcc::encoder {

#define JPCC_ENCODER_TMC2_OPT_PREFIX "jpccEncoderTMC2Parameter"
class JPCCEncoderTMC2Parameter : public virtual Parameter {
 public:
  PCCEncoderTMC2Parameters encoderParams_;

 public:
  JPCCEncoderTMC2Parameter();

  JPCCEncoderTMC2Parameter(const std::string& prefix, const std::string& caption);

  void setDefault();

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const JPCCEncoderTMC2Parameter& obj);
};

static std::istream& operator>>(std::istream& in, PCCColorTransform& val);
static std::istream& operator>>(std::istream& in, PCCCodecId& val);
static std::ostream& operator<<(std::ostream& out, const PCCColorTransform& val);
static std::ostream& operator<<(std::ostream& out, const PCCCodecId& val);

}  // namespace jpcc::encoder
