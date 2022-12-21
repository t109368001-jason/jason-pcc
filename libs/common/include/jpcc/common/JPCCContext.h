#pragma once

#include <jpcc/common/CoderBackendType.h>
#include <jpcc/common/Common.h>
#include <jpcc/common/IJPCCCombinationContext.h>
#include <jpcc/common/IJPCCEncoderContext.h>
#include <jpcc/common/IJPCCDecoderContext.h>
#include <jpcc/common/IJPCCSegmentationContext.h>
#include <jpcc/common/JPCCHeader.h>
#include <jpcc/common/SegmentationType.h>
#include <jpcc/common/SegmentationOutputType.h>

namespace jpcc {

class JPCCContext : public IJPCCSegmentationContext,
                    public IJPCCEncoderContext,
                    public IJPCCDecoderContext,
                    public IJPCCCombinationContext {
 protected:
  JPCCHeader header_;
  Index      startFrameNumber_;

  GroupOfFrame frames_;

  // for segmentation
  GroupOfPclFrame<PointSegmentation> pclFrames_;

  GroupOfFrame dynamicFrames_;
  GroupOfFrame staticFrames_;
  GroupOfFrame staticAddedFrames_;
  GroupOfFrame staticRemovedFrames_;

  // coder specified type
  std::vector<std::shared_ptr<void>> coderDynamicFrames_;
  std::vector<std::shared_ptr<void>> coderStaticFrames_;
  std::vector<std::shared_ptr<void>> coderStaticAddedFrames_;
  std::vector<std::shared_ptr<void>> coderStaticRemovedFrames_;

  std::vector<std::vector<char>> dynamicEncodedBytesVector_;
  std::vector<std::vector<char>> staticEncodedBytesVector_;
  std::vector<std::vector<char>> staticAddedEncodedBytesVector_;
  std::vector<std::vector<char>> staticRemovedEncodedBytesVector_;

  // coder specified type
  std::vector<std::shared_ptr<void>> coderDynamicReconstructFrames_;
  std::vector<std::shared_ptr<void>> coderStaticReconstructFrames_;
  std::vector<std::shared_ptr<void>> coderStaticAddedReconstructFrames_;
  std::vector<std::shared_ptr<void>> coderStaticRemovedReconstructFrames_;

  GroupOfFrame dynamicReconstructFrames_;
  GroupOfFrame staticReconstructFrames_;
  GroupOfFrame staticAddedReconstructFrames_;
  GroupOfFrame staticRemovedReconstructFrames_;

  GroupOfPclFrame<PointCombination> staticAddedReconstructPclFrames_;
  GroupOfPclFrame<PointCombination> staticRemovedReconstructPclFrames_;

  GroupOfFrame reconstructFrames_;

 public:
  JPCCContext(double                 resolution,
              SegmentationType       segmentationType,
              SegmentationOutputType segmentationOutputType,
              CoderBackendType       dynamicBackendType,
              CoderBackendType       staticBackendType);

  JPCCContext(JPCCHeader header);  // NOLINT(google-explicit-constructor)

  void clear();

  [[nodiscard]] JPCCHeader             getHeader() const { return header_; };
  [[nodiscard]] SegmentationType       getSegmentationType() const override { return header_.segmentationType; };
  [[nodiscard]] SegmentationOutputType getSegmentationOutputType() const override {
    return header_.segmentationOutputType;
  };
  [[nodiscard]] Index               getStartFrameNumber() const override { return startFrameNumber_; };
  [[nodiscard]] const GroupOfFrame& getFrames() const override { return frames_; };
  [[nodiscard]] const GroupOfPclFrame<PointSegmentation>& getPclFrames() const override { return pclFrames_; };

  [[nodiscard]] const GroupOfFrame& getDynamicFrames() const override { return dynamicFrames_; };
  [[nodiscard]] const GroupOfFrame& getStaticFrames() const override { return staticFrames_; };
  [[nodiscard]] const GroupOfFrame& getStaticAddedFrames() const override { return staticAddedFrames_; };
  [[nodiscard]] const GroupOfFrame& getStaticRemovedFrames() const override { return staticRemovedFrames_; };
  [[nodiscard]] const std::vector<std::shared_ptr<void>>& getCoderDynamicFrames() const override {
    return coderDynamicFrames_;
  }
  [[nodiscard]] const std::vector<std::shared_ptr<void>>& getCoderStaticFrames() const override {
    return coderStaticFrames_;
  }
  [[nodiscard]] const std::vector<std::shared_ptr<void>>& getCoderStaticAddedFrames() const override {
    return coderStaticAddedFrames_;
  }
  [[nodiscard]] const std::vector<std::shared_ptr<void>>& getCoderStaticRemovedFrames() const override {
    return coderStaticRemovedFrames_;
  }
  [[nodiscard]] const std::vector<std::vector<char>>& getDynamicEncodedBytesVector() const override {
    return dynamicEncodedBytesVector_;
  };
  [[nodiscard]] const std::vector<std::vector<char>>& getStaticEncodedBytesVector() const override {
    return staticEncodedBytesVector_;
  };
  [[nodiscard]] const std::vector<std::vector<char>>& getStaticAddedEncodedBytesVector() const override {
    return staticAddedEncodedBytesVector_;
  };
  [[nodiscard]] const std::vector<std::vector<char>>& getStaticRemovedEncodedBytesVector() const override {
    return staticRemovedEncodedBytesVector_;
  };
  [[nodiscard]] const std::vector<std::shared_ptr<void>>& getCoderDynamicReconstructFrames() const override {
    return coderDynamicReconstructFrames_;
  }
  [[nodiscard]] const std::vector<std::shared_ptr<void>>& getCoderStaticReconstructFrames() const override {
    return coderStaticReconstructFrames_;
  }
  [[nodiscard]] const std::vector<std::shared_ptr<void>>& getCoderStaticAddedReconstructFrames() const override {
    return coderStaticAddedReconstructFrames_;
  }
  [[nodiscard]] const std::vector<std::shared_ptr<void>>& getCoderStaticRemovedReconstructFrames() const override {
    return coderStaticRemovedReconstructFrames_;
  }
  [[nodiscard]] const GroupOfFrame& getDynamicReconstructFrames() const override { return dynamicReconstructFrames_; };
  [[nodiscard]] const GroupOfFrame& getStaticReconstructFrames() const override { return staticReconstructFrames_; };
  [[nodiscard]] const GroupOfFrame& getStaticAddedReconstructFrames() const override {
    return staticAddedReconstructFrames_;
  };
  [[nodiscard]] const GroupOfFrame& getStaticRemovedReconstructFrames() const override {
    return staticRemovedReconstructFrames_;
  };
  [[nodiscard]] const GroupOfPclFrame<PointCombination>& getStaticAddedReconstructPclFrames() const override {
    return staticAddedReconstructPclFrames_;
  };
  [[nodiscard]] const GroupOfPclFrame<PointCombination>& getStaticRemovedReconstructPclFrames() const override {
    return staticRemovedReconstructPclFrames_;
  };
  [[nodiscard]] const GroupOfFrame& getReconstructFrames() const override { return reconstructFrames_; };

  [[nodiscard]] Index&                              getStartFrameNumber() override { return startFrameNumber_; };
  [[nodiscard]] GroupOfFrame&                       getFrames() override { return frames_; };
  [[nodiscard]] GroupOfPclFrame<PointSegmentation>& getPclFrames() override { return pclFrames_; };
  [[nodiscard]] GroupOfFrame&                       getDynamicFrames() override { return dynamicFrames_; };
  [[nodiscard]] GroupOfFrame&                       getStaticFrames() override { return staticFrames_; };
  [[nodiscard]] GroupOfFrame&                       getStaticAddedFrames() override { return staticAddedFrames_; };
  [[nodiscard]] GroupOfFrame&                       getStaticRemovedFrames() override { return staticRemovedFrames_; };
  [[nodiscard]] std::vector<std::shared_ptr<void>>& getCoderDynamicFrames() override { return coderDynamicFrames_; }
  [[nodiscard]] std::vector<std::shared_ptr<void>>& getCoderStaticFrames() override { return coderStaticFrames_; }
  [[nodiscard]] std::vector<std::shared_ptr<void>>& getCoderStaticAddedFrames() override {
    return coderStaticAddedFrames_;
  }
  [[nodiscard]] std::vector<std::shared_ptr<void>>& getCoderStaticRemovedFrames() override {
    return coderStaticRemovedFrames_;
  }
  [[nodiscard]] std::vector<std::vector<char>>& getDynamicEncodedBytesVector() override {
    return dynamicEncodedBytesVector_;
  };
  [[nodiscard]] std::vector<std::vector<char>>& getStaticEncodedBytesVector() override {
    return staticEncodedBytesVector_;
  };
  [[nodiscard]] std::vector<std::vector<char>>& getStaticAddedEncodedBytesVector() override {
    return staticAddedEncodedBytesVector_;
  };
  [[nodiscard]] std::vector<std::vector<char>>& getStaticRemovedEncodedBytesVector() override {
    return staticRemovedEncodedBytesVector_;
  };
  [[nodiscard]] std::vector<std::shared_ptr<void>>& getCoderDynamicReconstructFrames() override {
    return coderDynamicReconstructFrames_;
  }
  [[nodiscard]] std::vector<std::shared_ptr<void>>& getCoderStaticReconstructFrames() override {
    return coderStaticReconstructFrames_;
  }
  [[nodiscard]] std::vector<std::shared_ptr<void>>& getCoderStaticAddedReconstructFrames() override {
    return coderStaticAddedReconstructFrames_;
  }
  [[nodiscard]] std::vector<std::shared_ptr<void>>& getCoderStaticRemovedReconstructFrames() override {
    return coderStaticRemovedReconstructFrames_;
  }
  [[nodiscard]] GroupOfFrame& getDynamicReconstructFrames() override { return dynamicReconstructFrames_; };
  [[nodiscard]] GroupOfFrame& getStaticReconstructFrames() override { return staticReconstructFrames_; };
  [[nodiscard]] GroupOfFrame& getStaticAddedReconstructFrames() override { return staticAddedReconstructFrames_; };
  [[nodiscard]] GroupOfFrame& getStaticRemovedReconstructFrames() override { return staticRemovedReconstructFrames_; };
  [[nodiscard]] GroupOfPclFrame<PointCombination>& getStaticAddedReconstructPclFrames() override {
    return staticAddedReconstructPclFrames_;
  };
  [[nodiscard]] GroupOfPclFrame<PointCombination>& getStaticRemovedReconstructPclFrames() override {
    return staticRemovedReconstructPclFrames_;
  };
  [[nodiscard]] GroupOfFrame& getReconstructFrames() override { return reconstructFrames_; };

  void convertToPclBuild(bool parallel);

  void convertToPclCombination(bool parallel);
};

void writeJPCCContext(const JPCCContext& context, std::ostream& os);

}  // namespace jpcc
