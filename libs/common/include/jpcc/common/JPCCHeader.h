#pragma once

#include <istream>
#include <ostream>

#include <jpcc/common/CoderBackendType.h>
#include <jpcc/common/SegmentationType.h>
#include <jpcc/common/SegmentationOutputType.h>

namespace jpcc {

struct JPCCHeader {
  SegmentationType       segmentationType;
  SegmentationOutputType segmentationOutputType;

  CoderBackendType dynamicBackendType;
  CoderBackendType staticBackendType;

  bool operator==(const JPCCHeader& other) const {
    if (this->segmentationType != other.segmentationType) { return false; }
    if (this->segmentationOutputType != other.segmentationOutputType) { return false; }
    if (this->dynamicBackendType != other.dynamicBackendType) { return false; }
    if (this->staticBackendType != other.staticBackendType) { return false; }
    return true;
  }
};

std::ostream& writeJPCCHeader(const JPCCHeader& header, std::ostream& os);

std::istream& readJPCCHeader(std::istream& is, JPCCHeader* header);

}  // namespace jpcc
