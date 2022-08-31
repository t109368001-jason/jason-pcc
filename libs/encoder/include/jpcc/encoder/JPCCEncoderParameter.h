#pragma once

#include <iostream>
#include <string>

#include <jpcc/common/Parameter.h>

namespace jpcc::encoder {

enum class EncoderBackendType { NONE, TMC3 };

#define JPCC_ENCODER_OPT_PREFIX "jpccEncoderParameter"

class JPCCEncoderParameter : public virtual Parameter {
 protected:
  std::string backendType_;

 public:
  EncoderBackendType backendType;

  JPCCEncoderParameter();

  JPCCEncoderParameter(const std::string& prefix, const std::string& caption);

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const JPCCEncoderParameter& obj);
};

[[nodiscard]] EncoderBackendType getEncoderBackendType(const std::string& encoderBackendType);
}  // namespace jpcc::encoder