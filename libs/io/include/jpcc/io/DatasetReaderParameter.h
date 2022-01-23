#pragma once

#include <jpcc/io/ReaderParameter.h>
#include <jpcc/io/LvxReaderParameter.h>

namespace jpcc::io {

class DatasetReaderParameter : public ReaderParameter, public LvxReaderParameter {
 public:
  DatasetReaderParameter();

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const DatasetReaderParameter& obj);
};

}  // namespace jpcc::io
