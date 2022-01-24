#include <jpcc/io/DatasetReaderParameter.h>

namespace jpcc::io {

using namespace std;
using namespace po;

#define READER_OPT_PREFIX "reader"

DatasetReaderParameter::DatasetReaderParameter() : DatasetReaderParameter(READER_OPT_PREFIX, __FUNCTION__) {}

DatasetReaderParameter::DatasetReaderParameter(const std::string& prefix, const std::string& caption) :
    ReaderParameter(prefix, caption), LvxReaderParameter(prefix, caption), Parameter(prefix, caption) {}

void DatasetReaderParameter::notify() {
  ReaderParameter::notify();
  LvxReaderParameter::notify();
}

ostream& operator<<(ostream& out, const DatasetReaderParameter& obj) {
  out << *(ReaderParameter*)(&obj);
  out << *(LvxReaderParameter*)(&obj);
  return out;
}

}  // namespace jpcc::io
