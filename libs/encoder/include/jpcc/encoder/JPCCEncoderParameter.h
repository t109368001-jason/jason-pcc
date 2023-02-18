#pragma once

#include <iostream>
#include <string>

#include <jpcc/common/Parameter.h>
#include <jpcc/common/CoderBackendType.h>
#if defined(HAVE_MPEG_PCC_TMC2)
#include <jpcc/encoder/JPCCEncoderTMC2Parameter.h>
#endif
#include <jpcc/encoder/JPCCEncoderTMC3Parameter.h>

namespace jpcc::encoder {

#define JPCC_ENCODER_OPT_PREFIX "jpccEncoderParameter"

class JPCCEncoderParameter : public virtual Parameter {
 protected:
  std::string backendType_;

 public:
  CoderBackendType backendType;
#if defined(HAVE_MPEG_PCC_TMC2)
  JPCCEncoderTMC2Parameter tmc2;
#endif
  JPCCEncoderTMC3Parameter tmc3;

  JPCCEncoderParameter();

  JPCCEncoderParameter(const std::string& prefix, const std::string& caption);

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const JPCCEncoderParameter& obj);
};

}  // namespace jpcc::encoder