#pragma once

#include <iostream>

#include <jpcc/io/ReaderParameterBase.h>

namespace jpcc::io {

class LvxReaderParameter : public jpcc::io::ReaderParameterBase {
 public:
  float frequency;
  float interval;  // ms

  LvxReaderParameter();

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const LvxReaderParameter& obj);
};

}  // namespace jpcc::io
