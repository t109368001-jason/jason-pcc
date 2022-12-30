#pragma once

#include <istream>
#include <ostream>

#include <jpcc/common/CoderBackendType.h>
#include <jpcc/common/SegmentationType.h>
#include <jpcc/common/SegmentationOutputType.h>

namespace jpcc {

struct JPCCHeader {
  double resolution;

  CoderBackendType backendType;

  void write(std::ostream& os) const;

  void read(std::istream& is);

  bool operator==(const JPCCHeader& other) const;

  friend std::ostream& operator<<(std::ostream& out, const JPCCHeader& obj);
};

}  // namespace jpcc
