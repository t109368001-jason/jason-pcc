#pragma once

#include <iostream>
#include <string>

#include <jpcc/common/Parameter.h>

namespace jpcc::segmentation {

#define JPCC_GMM_SEGMENTATION_OPT_PREFIX "jpccGMMSegmentationParameter"

class JPCCGMMSegmentationParameter : public virtual Parameter {
 public:
  double resolution;
  int    k;
  double alpha;
  int    nTrain;
  double dynamicThresholdLE;
  double staticThresholdGT;

  JPCCGMMSegmentationParameter();

  JPCCGMMSegmentationParameter(const std::string& prefix, const std::string& caption);

  void getShowTexts(std::vector<std::string>& showTexts) const override;

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const JPCCGMMSegmentationParameter& obj);
};

}  // namespace jpcc::segmentation
