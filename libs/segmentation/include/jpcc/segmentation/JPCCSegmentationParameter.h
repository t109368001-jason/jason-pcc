#pragma once

#include <iostream>
#include <string>

#include <jpcc/common/Parameter.h>

namespace jpcc::segmentation {

enum class SegmentationType { GMM };
enum class StaticPointType { CENTER };

#define JPCC_GMM_SEGMENTATION_OPT_PREFIX "jpccGMMSegmentationParameter"

class JPCCSegmentationParameter : public virtual Parameter {
 protected:
  std::string         type_;
  std::string         staticPointType_;
  std::vector<bool>   outputExistsPointOnlyVector_;
  std::vector<int>    kVector_;
  std::vector<double> alphaVector_;
  std::vector<double> nullAlphaRatioVector_;
  std::vector<int>    nTrainVector_;
  std::vector<double> dynamicThresholdLEVector_;
  std::vector<double> staticThresholdGTVector_;

 public:
  SegmentationType type;
  StaticPointType  staticPointType;
  bool             updateModelBeforeNTrain;
  double           resolution;
  double           minimumVariance;

  JPCCSegmentationParameter();

  JPCCSegmentationParameter(const std::string& prefix, const std::string& caption);

  void getShowTexts(std::vector<std::string>& showTexts) const override;

  void notify() override;

  [[nodiscard]] bool   getOutputExistsPointOnly() const;
  [[nodiscard]] int    getK() const;
  [[nodiscard]] double getAlpha() const;
  [[nodiscard]] double getNullAlphaRatio() const;
  [[nodiscard]] int    getNTrain() const;
  [[nodiscard]] double getDynamicThresholdLE() const;
  [[nodiscard]] double getStaticThresholdGT() const;

  [[nodiscard]] bool   getOutputExistsPointOnly(int index) const;
  [[nodiscard]] int    getK(int index) const;
  [[nodiscard]] double getAlpha(int index) const;
  [[nodiscard]] double getNullAlphaRatio(int index) const;
  [[nodiscard]] int    getNTrain(int index) const;
  [[nodiscard]] double getDynamicThresholdLE(int index) const;
  [[nodiscard]] double getStaticThresholdGT(int index) const;

  [[nodiscard]] const std::vector<bool>&   getOutputExistsPointOnlyVector() const;
  [[nodiscard]] const std::vector<int>&    getKVector() const;
  [[nodiscard]] const std::vector<double>& getAlphaVector() const;
  [[nodiscard]] const std::vector<double>& getNullAlphaRatioVector() const;
  [[nodiscard]] const std::vector<int>&    getNTrainVector() const;
  [[nodiscard]] const std::vector<double>& getDynamicThresholdLEVector() const;
  [[nodiscard]] const std::vector<double>& getStaticThresholdGTVector() const;

  friend std::ostream& operator<<(std::ostream& out, const JPCCSegmentationParameter& obj);
};

SegmentationType getSegmentationType(const std::string& type);

StaticPointType getStaticPointType(const std::string& type);

}  // namespace jpcc::segmentation
