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
  std::string type_;
  std::string staticPointType_;

 public:
  SegmentationType type;
  StaticPointType  staticPointType;
  bool             updateModelBeforeNTrain;
  bool             outputExistsPointOnly;
  double           resolution;
  int              k;
  double           alpha;
  double           nullAlphaRatio;
  int              nTrain;
  double           dynamicThresholdLE;
  double           staticThresholdGT;
  double           minimumVariance;

  JPCCSegmentationParameter();

  JPCCSegmentationParameter(const std::string& prefix, const std::string& caption);

  void getShowTexts(std::vector<std::string>& showTexts) const override;

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const JPCCSegmentationParameter& obj);
};

SegmentationType getSegmentationType(const std::string& type);

StaticPointType getStaticPointType(const std::string& type);

}  // namespace jpcc::segmentation
