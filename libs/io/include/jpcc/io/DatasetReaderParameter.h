#pragma once

#include <jpcc/io/LvxReaderParameter.h>

namespace jpcc::io {

class DatasetReaderParameter : public virtual LvxReaderParameter {
 public:
  DatasetReaderParameter();

  DatasetReaderParameter(const std::string& prefix, const std::string& caption);

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const DatasetReaderParameter& obj);
};

}  // namespace jpcc::io
