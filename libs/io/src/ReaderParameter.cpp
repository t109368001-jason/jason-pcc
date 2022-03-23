#include <jpcc/io/ReaderParameter.h>

namespace jpcc::io {

using namespace std;
using namespace po;

#define READER_OPT_PREFIX "reader"

ReaderParameter::ReaderParameter() : ReaderParameter(READER_OPT_PREFIX, __FUNCTION__) {}

ReaderParameter::ReaderParameter(const std::string& prefix, const std::string& caption) : Parameter(prefix, caption) {
  opts_.add_options()  //
      ;
}

void ReaderParameter::notify() {}

ostream& operator<<(ostream& out, const ReaderParameter& obj) { return out; }

}  // namespace jpcc::io
