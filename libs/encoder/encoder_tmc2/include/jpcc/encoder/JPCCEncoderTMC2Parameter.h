#pragma once

#include <iostream>
#include <string>

#include <jpcc/common/Parameter.h>

namespace jpcc::encoder {

#define JPCC_ENCODER_TMC2_OPT_PREFIX "jpccEncoderTMC2Parameter"

class JPCCEncoderTMC2Parameter : public virtual Parameter {
 protected:
  po::options_description tmc2Opts_;

 public:
  std::shared_ptr<void>    params_;
  std::vector<std::string> tmc2Configs_;

 public:
  JPCCEncoderTMC2Parameter();

  JPCCEncoderTMC2Parameter(const std::string& prefix, const std::string& caption);

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const JPCCEncoderTMC2Parameter& obj);
};

}  // namespace jpcc::encoder
