#pragma once

#include <fstream>

#include <jpcc/common/Common.h>
#include <jpcc/common/JPCCCoderContext.h>
#include <jpcc/common/IJPCCCombinationContext.h>
#include <jpcc/common/IJPCCSegmentationContext.h>
#include <jpcc/common/SegmentationType.h>
#include <jpcc/common/SegmentationOutputType.h>

namespace jpcc {

class JPCCContext : public IJPCCSegmentationContext, public IJPCCCombinationContext {
 protected:
  std::string            compressedStreamPathPrefix_;
  Index                  startFrameNumber_;
  SegmentationOutputType segmentationOutputType_;

  GroupOfFrame frames_;

  // for segmentation
  GroupOfPclFrame<PointSegmentation> pclFrames_;

  JPCCCoderContext::Ptr dynamic_;
  JPCCCoderContext::Ptr static_;
  JPCCCoderContext::Ptr staticAdded_;
  JPCCCoderContext::Ptr staticRemoved_;

  // for combination
  GroupOfPclFrame<PointCombination> staticAddedPclFrames_;
  GroupOfPclFrame<PointCombination> staticRemovedPclFrames_;

 public:
  JPCCContext(const std::string& compressedStreamPathPrefix);

  void clear();

  void writeHeader(SegmentationOutputType segmentationOutputType,
                   double                 resolution,
                   CoderBackendType       dynamicBackendType,
                   CoderBackendType       staticBackendType);

  void readHeader();

  void flush();

  void ifsSeekgEnd();

  [[nodiscard]] Index                   getStartFrameNumber() const { return startFrameNumber_; };
  [[nodiscard]] SegmentationOutputType  getSegmentationOutputType() const override { return segmentationOutputType_; };
  [[nodiscard]] const JPCCCoderContext& getDynamicContext() const { return *dynamic_; };
  [[nodiscard]] const JPCCCoderContext& getStaticContext() const { return *static_; };
  [[nodiscard]] const JPCCCoderContext& getStaticAddedContext() const { return *staticAdded_; };
  [[nodiscard]] const JPCCCoderContext& getStaticRemovedContext() const { return *staticRemoved_; };

  [[nodiscard]] const GroupOfFrame&                       getFrames() const override { return frames_; };
  [[nodiscard]] const GroupOfPclFrame<PointSegmentation>& getPclFrames() const override { return pclFrames_; };
  [[nodiscard]] const GroupOfFrame& getDynamicFrames() const override { return dynamic_->getFrames(); }
  [[nodiscard]] const GroupOfFrame& getStaticFrames() const override { return static_->getFrames(); }
  [[nodiscard]] const GroupOfFrame& getStaticAddedFrames() const override { return staticAdded_->getFrames(); }
  [[nodiscard]] const GroupOfFrame& getStaticRemovedFrames() const override { return staticRemoved_->getFrames(); }
  [[nodiscard]] const GroupOfPclFrame<PointCombination>& getStaticAddedReconstructPclFrames() const override {
    return staticAddedPclFrames_;
  };
  [[nodiscard]] const GroupOfPclFrame<PointCombination>& getStaticRemovedReconstructPclFrames() const override {
    return staticRemovedPclFrames_;
  };

  [[nodiscard]] Index&                              getStartFrameNumber() { return startFrameNumber_; };
  [[nodiscard]] JPCCCoderContext&                   getDynamicContext() { return *dynamic_; };
  [[nodiscard]] JPCCCoderContext&                   getStaticContext() { return *static_; };
  [[nodiscard]] JPCCCoderContext&                   getStaticAddedContext() { return *staticAdded_; };
  [[nodiscard]] JPCCCoderContext&                   getStaticRemovedContext() { return *staticRemoved_; };
  [[nodiscard]] GroupOfFrame&                       getFrames() override { return frames_; };
  [[nodiscard]] GroupOfPclFrame<PointSegmentation>& getPclFrames() override { return pclFrames_; };
  [[nodiscard]] GroupOfFrame&                       getDynamicFrames() override { return dynamic_->getFrames(); };
  [[nodiscard]] GroupOfFrame&                       getStaticFrames() override { return static_->getFrames(); };
  [[nodiscard]] GroupOfFrame& getStaticAddedFrames() override { return staticAdded_->getFrames(); };
  [[nodiscard]] GroupOfFrame& getStaticRemovedFrames() override { return staticRemoved_->getFrames(); };
  [[nodiscard]] GroupOfPclFrame<PointCombination>& getStaticAddedPclFrames() override { return staticAddedPclFrames_; };
  [[nodiscard]] GroupOfPclFrame<PointCombination>& getStaticRemovedPclFrames() override {
    return staticRemovedPclFrames_;
  };

  void convertToPclBuild(bool parallel);

  void convertToPclCombination(bool parallel);
};

}  // namespace jpcc
