#pragma once

#include <iostream>
#include <string>

#include <jpcc/common/Parameter.h>

namespace jpcc::encoder {

#define JPCC_ENCODER_TMC3_OPT_PREFIX "jpccEncoderTMC3Parameter"

class JPCCEncoderTMC3Parameter : public virtual Parameter {
 public:
  std::shared_ptr<void> params_;

 public:
  JPCCEncoderTMC3Parameter();

  JPCCEncoderTMC3Parameter(const std::string& prefix, const std::string& caption);

  void setDefault();

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const JPCCEncoderTMC3Parameter& obj);
};

}  // namespace jpcc::encoder
