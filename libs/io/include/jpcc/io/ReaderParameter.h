#pragma once

#include <iostream>
#include <set>
#include <string>
#include <vector>

#include <jpcc/common/Parameter.h>

namespace jpcc::io {

namespace po = boost::program_options;

class ReaderParameter : public virtual jpcc::common::Parameter {
 public:
  using Ptr = shared_ptr<ReaderParameter>;

 protected:
  std::vector<std::string> pointTypes_;

 public:
  std::set<std::string> pointTypes;
  float                 epsilon;

  ReaderParameter();

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const ReaderParameter& obj);
};

}  // namespace jpcc::io
