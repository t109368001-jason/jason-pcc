#pragma once

#include <iostream>
#include <string>

#include <jpcc/common/Parameter.h>

namespace jpcc::encoder {

#define JPCC_ENCODER_TMC2_OPT_PREFIX "jpccEncoderTMC2Parameter"

class JPCCEncoderTMC2Parameter : public virtual Parameter {
 public:
  JPCCEncoderTMC2Parameter();

  JPCCEncoderTMC2Parameter(const std::string& prefix, const std::string& caption);

  void setDefault();

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const JPCCEncoderTMC2Parameter& obj);
};

}  // namespace jpcc::encoder
