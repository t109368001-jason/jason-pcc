#include <jpcc/segmentation/JPCCGMMSegmentationParameter.h>

namespace jpcc::segmentation {

using namespace std;
using namespace po;

#define RESOLUTION_OPT ".resolution"
#define K_OPT ".k"
#define ALPHA_OPT ".alpha"
#define N_TRAIN_OPT ".nTrain"
#define DYNAMIC_THRESHOLD_OPT ".dynamicThresholdLE"
#define STATIC_THRESHOLD_OPT ".staticThresholdGT"

JPCCGMMSegmentationParameter::JPCCGMMSegmentationParameter() :
    JPCCGMMSegmentationParameter(JPCC_GMM_SEGMENTATION_OPT_PREFIX, __FUNCTION__) {}

JPCCGMMSegmentationParameter::JPCCGMMSegmentationParameter(const string& prefix, const string& caption) :
    Parameter(prefix, caption), resolution(0.0), k(3), alpha(0.05) {
  opts_.add_options()                                           //
      (string(prefix_ + RESOLUTION_OPT).c_str(),                //
       value<double>(&resolution)->required(),                  //
       JPCC_GMM_SEGMENTATION_OPT_PREFIX " resolution")          //
      (string(prefix_ + K_OPT).c_str(),                         //
       value<int>(&k)->required(),                              //
       JPCC_GMM_SEGMENTATION_OPT_PREFIX " k")                   //
      (string(prefix_ + ALPHA_OPT).c_str(),                     //
       value<double>(&alpha)->required(),                       //
       JPCC_GMM_SEGMENTATION_OPT_PREFIX " alpha")               //
      (string(prefix_ + N_TRAIN_OPT).c_str(),                   //
       value<int>(&nTrain)->required(),                         //
       JPCC_GMM_SEGMENTATION_OPT_PREFIX " nTrain")              //
      (string(prefix_ + DYNAMIC_THRESHOLD_OPT).c_str(),         //
       value<double>(&dynamicThresholdLE)->required(),          //
       JPCC_GMM_SEGMENTATION_OPT_PREFIX " dynamicThresholdLE")  //
      (string(prefix_ + STATIC_THRESHOLD_OPT).c_str(),          //
       value<double>(&staticThresholdGT)->required(),           //
       JPCC_GMM_SEGMENTATION_OPT_PREFIX " staticThresholdGT")   //
      ;
}

void JPCCGMMSegmentationParameter::getShowTexts(vector<string>& showTexts) const {
  showTexts.push_back(prefix_ + RESOLUTION_OPT ": " + to_string(resolution));
  showTexts.push_back(prefix_ + K_OPT ": " + to_string(k));
  showTexts.push_back(prefix_ + ALPHA_OPT ": " + to_string(alpha));
  showTexts.push_back(prefix_ + N_TRAIN_OPT ": " + to_string(nTrain));
  showTexts.push_back(prefix_ + DYNAMIC_THRESHOLD_OPT ": " + to_string(dynamicThresholdLE));
  showTexts.push_back(prefix_ + STATIC_THRESHOLD_OPT ": " + to_string(staticThresholdGT));
}

void JPCCGMMSegmentationParameter::notify() {
  assert(resolution > 0.0);
  assert(k > 0);
  assert(alpha > 0.0);
  assert(nTrain > 0.0);
}

ostream& operator<<(ostream& out, const JPCCGMMSegmentationParameter& obj) {
  obj.coutParameters(out)                              //
      (RESOLUTION_OPT, obj.resolution)                 //
      (K_OPT, obj.k)                                   //
      (ALPHA_OPT, obj.alpha)                           //
      (N_TRAIN_OPT, obj.nTrain)                        //
      (DYNAMIC_THRESHOLD_OPT, obj.dynamicThresholdLE)  //
      (STATIC_THRESHOLD_OPT, obj.staticThresholdGT)    //
      ;
  return out;
}

}  // namespace jpcc::segmentation
