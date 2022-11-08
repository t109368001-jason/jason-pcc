#include <jpcc/common/JPCCContext.h>

#include <execution>

#include <boost/range/counting_range.hpp>

namespace jpcc {

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCContext::JPCCContext(double                 resolution,
                         SegmentationType       segmentationType,
                         SegmentationOutputType segmentationOutputType,
                         CoderBackendType       dynamicBackendType,
                         CoderBackendType       staticBackendType) :
    header_({0}) {
  header_.resolution             = resolution;
  header_.segmentationType       = segmentationType;
  header_.segmentationOutputType = segmentationOutputType;
  header_.dynamicBackendType     = dynamicBackendType;
  header_.staticBackendType      = staticBackendType;
}

JPCCContext::JPCCContext(JPCCHeader header) : header_(header) {}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCContext::clear() {
  pclFrames_.clear();
  dynamicFrames_.clear();
  staticFrames_.clear();
  staticAddedFrames_.clear();
  staticRemovedFrames_.clear();
  dynamicEncodedBytesVector_.clear();
  staticEncodedBytesVector_.clear();
  staticAddedEncodedBytesVector_.clear();
  staticRemovedEncodedBytesVector_.clear();
  dynamicReconstructFrames_.clear();
  staticReconstructFrames_.clear();
  staticAddedReconstructFrames_.clear();
  staticRemovedReconstructFrames_.clear();
  reconstructFrames_.clear();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCContext::convertToPclBuild(bool parallel) {
  pclFrames_.resize(frames_.size());
  if (!parallel) {
    for (size_t i = 0; i < frames_.size(); i++) { pclFrames_[i] = frames_[i]->toPcl<PointSegmentation>(); }
  } else {
    const auto range = boost::counting_range<size_t>(0, frames_.size());
    std::for_each(std::execution::par, range.begin(), range.end(),
                  [&](const size_t& i) {  //
                    pclFrames_[i] = frames_[i]->toPcl<PointSegmentation>();
                  });
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCContext::convertToPclCombination(bool parallel) {
  staticAddedReconstructPclFrames_.resize(staticAddedReconstructFrames_.size());
  if (!parallel) {
    for (size_t i = 0; i < staticAddedReconstructFrames_.size(); i++) {
      staticAddedReconstructPclFrames_[i] = staticAddedReconstructFrames_[i]->toPcl<PointCombination>();
    }
  } else {
    const auto range = boost::counting_range<size_t>(0, staticAddedReconstructFrames_.size());
    std::for_each(std::execution::par, range.begin(), range.end(),
                  [&](const size_t& i) {  //
                    staticAddedReconstructPclFrames_[i] = staticAddedReconstructFrames_[i]->toPcl<PointCombination>();
                  });
  }
  staticRemovedReconstructPclFrames_.resize(staticRemovedReconstructFrames_.size());
  if (!parallel) {
    for (size_t i = 0; i < staticRemovedReconstructFrames_.size(); i++) {
      staticRemovedReconstructPclFrames_[i] = staticRemovedReconstructFrames_[i]->toPcl<PointCombination>();
    }
  } else {
    const auto range = boost::counting_range<size_t>(0, staticRemovedReconstructFrames_.size());
    std::for_each(std::execution::par, range.begin(), range.end(),
                  [&](const size_t& i) {  //
                    staticRemovedReconstructPclFrames_[i] =
                        staticRemovedReconstructFrames_[i]->toPcl<PointCombination>();
                  });
  }
}

void writeJPCCContext(const JPCCContext& context, std::ostream& os) {
  for (int i = 0; i < context.getDynamicEncodedBytesVector().size(); i++) {
    os.write(context.getDynamicEncodedBytesVector()[i].data(),
             (std::streamsize)context.getDynamicEncodedBytesVector()[i].size());
    if (context.getHeader().segmentationOutputType == jpcc::SegmentationOutputType::DYNAMIC_STATIC) {
      os.write(context.getStaticEncodedBytesVector()[i].data(),
               (std::streamsize)context.getStaticEncodedBytesVector()[i].size());
    } else if (context.getHeader().segmentationOutputType ==
               jpcc::SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED) {
      os.write(context.getStaticAddedEncodedBytesVector()[i].data(),
               (std::streamsize)context.getStaticAddedEncodedBytesVector()[i].size());
      os.write(context.getStaticRemovedEncodedBytesVector()[i].data(),
               (std::streamsize)context.getStaticRemovedEncodedBytesVector()[i].size());
    }
  }
}

}  // namespace jpcc