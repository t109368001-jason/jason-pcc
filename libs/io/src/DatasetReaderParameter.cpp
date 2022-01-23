#include <jpcc/io/DatasetReaderParameter.h>

namespace jpcc::io {

using namespace std;
using namespace po;

#define DATASET_READER_OPT_PREFIX "datasetReader"
#define FREQUENCY_OPT DATASET_READER_OPT_PREFIX ".frequency"

DatasetReaderParameter::DatasetReaderParameter() :
    ReaderParameter(), LvxReaderParameter(), Parameter(DATASET_READER_OPT_PREFIX, "DatasetReaderParameter") {}

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
