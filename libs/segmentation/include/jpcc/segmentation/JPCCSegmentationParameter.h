#pragma once

#include <iostream>
#include <string>

#include <jpcc/common/Parameter.h>
#include <jpcc/common/SegmentationOutputType.h>
#include <jpcc/common/SegmentationType.h>
#include <jpcc/segmentation/StaticPointType.h>

namespace jpcc::segmentation {

#define JPCC_GMM_SEGMENTATION_OPT_PREFIX "jpccGMMSegmentationParameter"

class JPCCSegmentationParameter : public virtual Parameter {
 protected:
  std::string         type_;
  std::string         outputType_;
  std::string         staticPointType_;
  std::vector<bool>   outputExistsPointOnlyVector_;
  std::vector<int>    kVector_;
  std::vector<double> alphaVector_;
  std::vector<double> nullAlphaRatioVector_;
  std::vector<int>    nTrainVector_;
  std::vector<double> staticThreshold1Vector_;
  std::vector<double> staticThreshold2Vector_;
  std::vector<double> nullStaticThreshold1Vector_;
  std::vector<double> nullStaticThreshold2Vector_;

 public:
  SegmentationType       type;
  SegmentationOutputType outputType;
  StaticPointType        staticPointType;
  bool                   updateModelBeforeNTrain;
  double                 resolution;
  double                 minimumVariance;

  JPCCSegmentationParameter();

  JPCCSegmentationParameter(const std::string& prefix, const std::string& caption);

  void getShowTexts(std::vector<std::string>& showTexts) const override;

  void notify() override;

  [[nodiscard]] bool   getOutputExistsPointOnly() const;
  [[nodiscard]] int    getK() const;
  [[nodiscard]] double getAlpha() const;
  [[nodiscard]] double getNullAlphaRatio() const;
  [[nodiscard]] int    getNTrain() const;
  [[nodiscard]] double getStaticThreshold1() const;
  [[nodiscard]] double getStaticThreshold2() const;
  [[nodiscard]] double getNullStaticThreshold1() const;
  [[nodiscard]] double getNullStaticThreshold2() const;

  [[nodiscard]] bool   getOutputExistsPointOnly(int index) const;
  [[nodiscard]] int    getK(int index) const;
  [[nodiscard]] double getAlpha(int index) const;
  [[nodiscard]] double getNullAlphaRatio(int index) const;
  [[nodiscard]] int    getNTrain(int index) const;
  [[nodiscard]] double getStaticThreshold1(int index) const;
  [[nodiscard]] double getStaticThreshold2(int index) const;
  [[nodiscard]] double getNullStaticThreshold1(int index) const;
  [[nodiscard]] double getNullStaticThreshold2(int index) const;

  [[nodiscard]] const std::vector<bool>&   getOutputExistsPointOnlyVector() const;
  [[nodiscard]] const std::vector<int>&    getKVector() const;
  [[nodiscard]] const std::vector<double>& getAlphaVector() const;
  [[nodiscard]] const std::vector<double>& getNullAlphaRatioVector() const;
  [[nodiscard]] const std::vector<int>&    getNTrainVector() const;
  [[nodiscard]] const std::vector<double>& getStaticThreshold1Vector() const;
  [[nodiscard]] const std::vector<double>& getStaticThreshold2Vector() const;
  [[nodiscard]] const std::vector<double>& getNullStaticThreshold1Vector() const;
  [[nodiscard]] const std::vector<double>& getNullStaticThreshold2Vector() const;

  friend std::ostream& operator<<(std::ostream& out, const JPCCSegmentationParameter& obj);
};

}  // namespace jpcc::segmentation
