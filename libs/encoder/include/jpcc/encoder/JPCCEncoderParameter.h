#pragma once

#include <iostream>
#include <string>

#include <jpcc/common/Parameter.h>
#include <jpcc/common/CoderBackendType.h>
#include <jpcc/encoder/JPCCEncoderTMC3Parameter.h>

namespace jpcc::encoder {

#define JPCC_ENCODER_OPT_PREFIX "jpccEncoderParameter"

class JPCCEncoderParameter : public virtual Parameter {
 protected:
  std::string backendType_;

 public:
  CoderBackendType         backendType;
  JPCCEncoderTMC3Parameter tmc3;

  JPCCEncoderParameter();

  JPCCEncoderParameter(const std::string& prefix, const std::string& caption);

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const JPCCEncoderParameter& obj);
};

}  // namespace jpcc::encoder