#pragma once

#include <iostream>
#include <set>
#include <string>
#include <vector>

#include <jpcc/common/Parameter.h>

namespace jpcc::io {

namespace po = boost::program_options;

class ReaderParameter : public virtual Parameter {
 public:
  ReaderParameter();

  ReaderParameter(const std::string& prefix, const std::string& caption);

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const ReaderParameter& obj);
};

}  // namespace jpcc::io
