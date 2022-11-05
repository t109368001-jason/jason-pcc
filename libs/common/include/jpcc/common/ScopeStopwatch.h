#pragma once

#include <jpcc/common/Common.h>

namespace jpcc {

class ScopeStopwatch {
 protected:
  Stopwatch& stopWatch_;

 public:
  ScopeStopwatch(Stopwatch& stopWatch);  // NOLINT(google-explicit-constructor)

  ~ScopeStopwatch();
};

}  // namespace jpcc
