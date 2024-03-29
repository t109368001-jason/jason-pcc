#include <jpcc/common/ScopeStopwatch.h>

namespace jpcc {

//////////////////////////////////////////////////////////////////////////////////////////////
ScopeStopwatch::ScopeStopwatch(Stopwatch& stopWatch) : stopWatch_(stopWatch) {
  stopWatch.start();
}

//////////////////////////////////////////////////////////////////////////////////////////////
ScopeStopwatch::~ScopeStopwatch() {
  stopWatch_.stop();
}

}  // namespace jpcc
