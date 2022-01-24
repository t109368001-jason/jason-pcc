#pragma once

#include <iostream>

#include <jpcc/common/Parameter.h>

namespace jpcc::io {

class LvxReaderParameter : public virtual jpcc::common::Parameter {
 public:
  float frequency;
  float interval;  // ms

  LvxReaderParameter();

  LvxReaderParameter(const std::string& prefix, const std::string& caption);

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const LvxReaderParameter& obj);
};

}  // namespace jpcc::io
