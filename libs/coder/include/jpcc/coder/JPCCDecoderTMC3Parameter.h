#pragma once

#include <iostream>
#include <string>

#include <PCCTMC3Decoder.h>

#include <jpcc/common/Parameter.h>

namespace jpcc::coder {

#define JPCC_DECODER_TMC3_OPT_PREFIX "jpccDecoderTMC3Parameter"

class JPCCDecoderTMC3Parameter : public virtual Parameter, public pcc::DecoderParams {
 public:
  JPCCDecoderTMC3Parameter();

  JPCCDecoderTMC3Parameter(const std::string& prefix, const std::string& caption);

  void setDefault();

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const JPCCDecoderTMC3Parameter& obj);
};

}  // namespace jpcc::coder
