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

  GroupOfFrame pclFrames_;

  GroupOfFrame dynamicPclFrames_;
  GroupOfFrame staticPclFrames_;
  GroupOfFrame staticAddedPclFrames_;
  GroupOfFrame staticRemovedPclFrames_;

  // coder specified type
  std::vector<shared_ptr<void>> dynamicFrames_;
  std::vector<shared_ptr<void>> staticFrames_;
  std::vector<shared_ptr<void>> staticAddedFrames_;
  std::vector<shared_ptr<void>> staticRemovedFrames_;

  std::vector<std::vector<char>> dynamicEncodedBytesVector_;
  std::vector<std::vector<char>> staticEncodedBytesVector_;
  std::vector<std::vector<char>> staticAddedEncodedBytesVector_;
  std::vector<std::vector<char>> staticRemovedEncodedBytesVector_;

  // coder specified type
  std::vector<shared_ptr<void>> dynamicReconstructFrames_;
  std::vector<shared_ptr<void>> staticReconstructFrames_;
  std::vector<shared_ptr<void>> staticAddedReconstructFrames_;
  std::vector<shared_ptr<void>> staticRemovedReconstructFrames_;

  GroupOfFrame dynamicReconstructPclFrames_;
  GroupOfFrame staticReconstructPclFrames_;
  GroupOfFrame staticAddedReconstructPclFrames_;
  GroupOfFrame staticRemovedReconstructPclFrames_;

  GroupOfFrame reconstructPclFrames_;

 public:
  JPCCContext(double                 resolution,
              SegmentationType       segmentationType,
              SegmentationOutputType segmentationOutputType,
              CoderBackendType       dynamicBackendType,
              CoderBackendType       staticBackendType);

  void clear();

  [[nodiscard]] JPCCHeader             getHeader() const { return header_; };
  [[nodiscard]] SegmentationType       getSegmentationType() const override { return header_.segmentationType; };
  [[nodiscard]] SegmentationOutputType getSegmentationOutputType() const override {
    return header_.segmentationOutputType;
  };
  [[nodiscard]] const GroupOfFrame& getPclFrames() const override { return pclFrames_; };
  [[nodiscard]] const GroupOfFrame& getDynamicPclFrames() const override { return dynamicPclFrames_; };
  [[nodiscard]] const GroupOfFrame& getStaticPclFrames() const override { return staticPclFrames_; };
  [[nodiscard]] const GroupOfFrame& getStaticAddedPclFrames() const override { return staticAddedPclFrames_; };
  [[nodiscard]] const GroupOfFrame& getStaticRemovedPclFrames() const override { return staticRemovedPclFrames_; };
  [[nodiscard]] const std::vector<shared_ptr<void>>& getDynamicFrames() const override { return dynamicFrames_; };
  [[nodiscard]] const std::vector<shared_ptr<void>>& getStaticFrames() const override { return staticFrames_; };
  [[nodiscard]] const std::vector<shared_ptr<void>>& getStaticAddedFrames() const override {
    return staticAddedFrames_;
  };
  [[nodiscard]] const std::vector<shared_ptr<void>>& getStaticRemovedFrames() const override {
    return staticRemovedFrames_;
  };
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
  [[nodiscard]] const std::vector<shared_ptr<void>>& getDynamicReconstructFrames() const override {
    return dynamicReconstructFrames_;
  };
  [[nodiscard]] const std::vector<shared_ptr<void>>& getStaticReconstructFrames() const override {
    return staticReconstructFrames_;
  };
  [[nodiscard]] const std::vector<shared_ptr<void>>& getStaticAddedReconstructFrames() const override {
    return staticAddedReconstructFrames_;
  };
  [[nodiscard]] const std::vector<shared_ptr<void>>& getStaticRemovedReconstructFrames() const override {
    return staticRemovedReconstructFrames_;
  };
  [[nodiscard]] const GroupOfFrame& getDynamicReconstructPclFrames() const override {
    return dynamicReconstructPclFrames_;
  };
  [[nodiscard]] const GroupOfFrame& getStaticReconstructPclFrames() const override {
    return staticReconstructPclFrames_;
  };
  [[nodiscard]] const GroupOfFrame& getStaticAddedReconstructPclFrames() const override {
    return staticAddedReconstructPclFrames_;
  };
  [[nodiscard]] const GroupOfFrame& getStaticRemovedReconstructPclFrames() const override {
    return staticRemovedReconstructPclFrames_;
  };
  [[nodiscard]] const GroupOfFrame& getReconstructPclFrames() const override { return reconstructPclFrames_; };

  [[nodiscard]] GroupOfFrame&                  getPclFrames() override { return pclFrames_; };
  [[nodiscard]] GroupOfFrame&                  getDynamicPclFrames() override { return dynamicPclFrames_; };
  [[nodiscard]] GroupOfFrame&                  getStaticPclFrames() override { return staticPclFrames_; };
  [[nodiscard]] GroupOfFrame&                  getStaticAddedPclFrames() override { return staticAddedPclFrames_; };
  [[nodiscard]] GroupOfFrame&                  getStaticRemovedPclFrames() override { return staticRemovedPclFrames_; };
  [[nodiscard]] std::vector<shared_ptr<void>>& getDynamicFrames() override { return dynamicFrames_; };
  [[nodiscard]] std::vector<shared_ptr<void>>& getStaticFrames() override { return staticFrames_; };
  [[nodiscard]] std::vector<shared_ptr<void>>& getStaticAddedFrames() override { return staticAddedFrames_; };
  [[nodiscard]] std::vector<shared_ptr<void>>& getStaticRemovedFrames() override { return staticRemovedFrames_; };
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
  [[nodiscard]] std::vector<shared_ptr<void>>& getDynamicReconstructFrames() override {
    return dynamicReconstructFrames_;
  };
  [[nodiscard]] std::vector<shared_ptr<void>>& getStaticReconstructFrames() override {
    return staticReconstructFrames_;
  };
  [[nodiscard]] std::vector<shared_ptr<void>>& getStaticAddedReconstructFrames() override {
    return staticAddedReconstructFrames_;
  };
  [[nodiscard]] std::vector<shared_ptr<void>>& getStaticRemovedReconstructFrames() override {
    return staticRemovedReconstructFrames_;
  };
  [[nodiscard]] GroupOfFrame& getDynamicReconstructPclFrames() override { return dynamicReconstructPclFrames_; };
  [[nodiscard]] GroupOfFrame& getStaticReconstructPclFrames() override { return staticReconstructPclFrames_; };
  [[nodiscard]] GroupOfFrame& getStaticAddedReconstructPclFrames() override {
    return staticAddedReconstructPclFrames_;
  };
  [[nodiscard]] GroupOfFrame& getStaticRemovedReconstructPclFrames() override {
    return staticRemovedReconstructPclFrames_;
  };
  [[nodiscard]] GroupOfFrame& getReconstructPclFrames() override { return reconstructPclFrames_; };
};

void writeJPCCContext(const JPCCContext& context, std::ostream& os);

}  // namespace jpcc
