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
};

std::ostream& writeJPCCHeader(const JPCCHeader& header, std::ostream& os);

std::istream& readJPCCHeader(std::istream& is, JPCCHeader* header);

}  // namespace jpcc
