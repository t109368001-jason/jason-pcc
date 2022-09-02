#pragma once

#include <iostream>
#include <string>

#include <jpcc/common/Parameter.h>
#include <jpcc/coder/CoderBackendType.h>
#include <jpcc/coder/JPCCDecoderTMC3Parameter.h>

namespace jpcc::coder {

#define JPCC_DECODER_OPT_PREFIX "jpccDecoderParameter"

class JPCCDecoderParameter : public virtual Parameter {
 protected:
  std::string backendType_;

 public:
  CoderBackendType         backendType;
  JPCCDecoderTMC3Parameter tmc3;

  JPCCDecoderParameter();

  JPCCDecoderParameter(const std::string& prefix, const std::string& caption);

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const JPCCDecoderParameter& obj);
};

}  // namespace jpcc::coder