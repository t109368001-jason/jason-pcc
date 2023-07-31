#include <jpcc/common/JPCCContext.h>

#include <execution>
#include <filesystem>

#include <boost/range/counting_range.hpp>

using namespace std::filesystem;

namespace jpcc {

JPCCContext::JPCCContext(const std::string& compressedStreamPathPrefix) :
    compressedStreamPathPrefix_(compressedStreamPathPrefix),
    segmentationOutputType_(),
    dynamic_(),
    static_(),
    staticAdded_(),
    staticRemoved_() {
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCContext::clear() {
  frames_.clear();
  pclFrames_.clear();
  dynamic_->clear();
  if (segmentationOutputType_ == SegmentationOutputType::DYNAMIC_STATIC) {
    static_->clear();
  } else if (segmentationOutputType_ == SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED) {
    staticAdded_->clear();
    staticRemoved_->clear();
  }
  staticAddedPclFrames_.clear();
  staticRemovedPclFrames_.clear();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCContext::writeHeader(SegmentationOutputType segmentationOutputType,
                              double                 resolution,
                              CoderBackendType       dynamicBackendType,
                              CoderBackendType       staticBackendType) {
  segmentationOutputType_ = segmentationOutputType;
  dynamic_                = std::make_shared<JPCCCoderContext>(compressedStreamPathPrefix_ + "-dynamic.bin");
  if (segmentationOutputType == SegmentationOutputType::DYNAMIC_STATIC) {
    static_ = std::make_shared<JPCCCoderContext>(compressedStreamPathPrefix_ + "-static.bin");
  } else if (segmentationOutputType == SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED) {
    staticAdded_   = std::make_shared<JPCCCoderContext>(compressedStreamPathPrefix_ + "-static-added.bin");
    staticRemoved_ = std::make_shared<JPCCCoderContext>(compressedStreamPathPrefix_ + "-static-removed.bin");
  }
  dynamic_->writeHeader(resolution, dynamicBackendType);
  if (segmentationOutputType_ == SegmentationOutputType::DYNAMIC_STATIC) {
    static_->writeHeader(resolution, staticBackendType);
  } else if (segmentationOutputType_ == SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED) {
    staticAdded_->writeHeader(resolution, staticBackendType);
    staticRemoved_->writeHeader(resolution, staticBackendType);
  }
  flush();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCContext::readHeader() {
  dynamic_ = std::make_shared<JPCCCoderContext>(compressedStreamPathPrefix_ + "-dynamic.bin");
  if (exists(compressedStreamPathPrefix_ + "-static.bin")) {
    segmentationOutputType_ = SegmentationOutputType::DYNAMIC_STATIC;
    static_                 = std::make_shared<JPCCCoderContext>(compressedStreamPathPrefix_ + "-static.bin");
  } else if (exists(compressedStreamPathPrefix_ + "-static-added.bin") &&
             exists(compressedStreamPathPrefix_ + "-static-removed.bin")) {
    segmentationOutputType_ = SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED;
    static_                 = std::make_shared<JPCCCoderContext>("");
    staticAdded_            = std::make_shared<JPCCCoderContext>(compressedStreamPathPrefix_ + "-static-added.bin");
    staticRemoved_          = std::make_shared<JPCCCoderContext>(compressedStreamPathPrefix_ + "-static-removed.bin");
  } else {
    segmentationOutputType_ = SegmentationOutputType::NONE;
  }
  dynamic_->readHeader();
  if (segmentationOutputType_ == SegmentationOutputType::DYNAMIC_STATIC) {
    static_->readHeader();
  } else if (segmentationOutputType_ == SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED) {
    staticAdded_->readHeader();
    staticRemoved_->readHeader();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCContext::flush() {
  dynamic_->flush();
  if (segmentationOutputType_ == SegmentationOutputType::DYNAMIC_STATIC) {
    static_->flush();
  } else if (segmentationOutputType_ == SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED) {
    staticAdded_->flush();
    staticRemoved_->flush();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCContext::ifsSeekgEnd() {
  dynamic_->ifsSeekgEnd();
  if (segmentationOutputType_ == SegmentationOutputType::DYNAMIC_STATIC) {
    static_->ifsSeekgEnd();
  } else if (segmentationOutputType_ == SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED) {
    staticAdded_->ifsSeekgEnd();
    staticRemoved_->ifsSeekgEnd();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool JPCCContext::anyEof() const {
  if (dynamic_->eof()) {
    return true;
  }
  if (segmentationOutputType_ == SegmentationOutputType::DYNAMIC_STATIC) {
    if (static_->eof()) {
      return true;
    }
  } else if (segmentationOutputType_ == SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED) {
    if (staticAdded_->eof()) {
      return true;
    }
    if (staticRemoved_->eof()) {
      return true;
    }
  }
  return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCContext::convertToPclBuild(bool parallel) {
  pclFrames_.resize(frames_.size());
  if (!parallel) {
    for (size_t i = 0; i < frames_.size(); i++) {
      pclFrames_[i] = frames_[i]->toPcl<PointSegmentation>();
    }
  } else {
    const auto range = boost::counting_range<size_t>(0, frames_.size());
    std::for_each(std::execution::par, range.begin(), range.end(),
                  [&](const size_t& i) {  //
                    pclFrames_[i] = frames_[i]->toPcl<PointSegmentation>();
                  });
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCContext::convertToPclCombination(bool parallel) {
  if (segmentationOutputType_ != SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED) {
    return;
  }
  staticAddedPclFrames_.resize(staticAdded_->getFrames().size());
  if (!parallel) {
    for (size_t i = 0; i < staticAdded_->getFrames().size(); i++) {
      staticAddedPclFrames_[i] = staticAdded_->getFrames()[i]->toPcl<PointCombination>();
    }
  } else {
    const auto range = boost::counting_range<size_t>(0, staticAdded_->getFrames().size());
    std::for_each(std::execution::par, range.begin(), range.end(),
                  [&](const size_t& i) {  //
                    staticAddedPclFrames_[i] = staticAdded_->getFrames()[i]->toPcl<PointCombination>();
                  });
  }
  staticRemovedPclFrames_.resize(staticRemoved_->getFrames().size());
  if (!parallel) {
    for (size_t i = 0; i < staticRemoved_->getFrames().size(); i++) {
      staticRemovedPclFrames_[i] = staticRemoved_->getFrames()[i]->toPcl<PointCombination>();
    }
  } else {
    const auto range = boost::counting_range<size_t>(0, staticRemoved_->getFrames().size());
    std::for_each(std::execution::par, range.begin(), range.end(),
                  [&](const size_t& i) {  //
                    staticRemovedPclFrames_[i] = staticRemoved_->getFrames()[i]->toPcl<PointCombination>();
                  });
  }
}

}  // namespace jpcc