#include <jpcc/segmentation/JPCCSegmentationParameter.h>

namespace jpcc::segmentation {

using namespace std;
using namespace po;

#define TYPE_OPT ".type"
#define STATIC_POINT_TYPE_OPT ".staticPointType"
#define UPDATE_MODEL_BEFORE_N_TRAIN_OPT ".updateModelBeforeNTrain"
#define RESOLUTION_OPT ".resolution"
#define K_OPT ".k"
#define ALPHA_OPT ".alpha"
#define NULL_ALPHA_RATIO_OPT ".nullAlphaRatio"
#define N_TRAIN_OPT ".nTrain"
#define DYNAMIC_THRESHOLD_OPT ".dynamicThresholdLE"
#define STATIC_THRESHOLD_OPT ".staticThresholdGT"
#define MINIMUM_VARIANCE_OPT ".minimumVariance"

JPCCSegmentationParameter::JPCCSegmentationParameter() :
    JPCCSegmentationParameter(JPCC_GMM_SEGMENTATION_OPT_PREFIX, __FUNCTION__) {}

JPCCSegmentationParameter::JPCCSegmentationParameter(const string& prefix, const string& caption) :
    Parameter(prefix, caption), type_(), staticPointType_(), resolution(0.0), k(3), alpha(0.05) {
  opts_.add_options()                                                //
      (string(prefix_ + TYPE_OPT).c_str(),                           //
       value<string>(&type_)->required(),                            //
       "type")                                                       //
      (string(prefix_ + STATIC_POINT_TYPE_OPT).c_str(),              //
       value<string>(&staticPointType_)->required(),                 //
       "staticPointType, [center]")                                  //
      (string(prefix_ + UPDATE_MODEL_BEFORE_N_TRAIN_OPT).c_str(),    //
       value<bool>(&updateModelBeforeNTrain)->required(),            //
       JPCC_GMM_SEGMENTATION_OPT_PREFIX " updateModelBeforeNTrain")  //
      (string(prefix_ + RESOLUTION_OPT).c_str(),                     //
       value<double>(&resolution)->required(),                       //
       JPCC_GMM_SEGMENTATION_OPT_PREFIX " resolution")               //
      (string(prefix_ + K_OPT).c_str(),                              //
       value<int>(&k)->required(),                                   //
       JPCC_GMM_SEGMENTATION_OPT_PREFIX " k")                        //
      (string(prefix_ + ALPHA_OPT).c_str(),                          //
       value<double>(&alpha)->required(),                            //
       JPCC_GMM_SEGMENTATION_OPT_PREFIX " alpha")                    //
      (string(prefix_ + NULL_ALPHA_RATIO_OPT).c_str(),               //
       value<double>(&nullAlphaRatio)->required(),                   //
       JPCC_GMM_SEGMENTATION_OPT_PREFIX " nullAlphaRatio")           //
      (string(prefix_ + N_TRAIN_OPT).c_str(),                        //
       value<int>(&nTrain)->required(),                              //
       JPCC_GMM_SEGMENTATION_OPT_PREFIX " nTrain")                   //
      (string(prefix_ + DYNAMIC_THRESHOLD_OPT).c_str(),              //
       value<double>(&dynamicThresholdLE)->required(),               //
       JPCC_GMM_SEGMENTATION_OPT_PREFIX " dynamicThresholdLE")       //
      (string(prefix_ + STATIC_THRESHOLD_OPT).c_str(),               //
       value<double>(&staticThresholdGT)->required(),                //
       JPCC_GMM_SEGMENTATION_OPT_PREFIX " staticThresholdGT")        //
      (string(prefix_ + MINIMUM_VARIANCE_OPT).c_str(),               //
       value<double>(&minimumVariance)->required(),                  //
       JPCC_GMM_SEGMENTATION_OPT_PREFIX " minimumVariance")          //
      ;
}

void JPCCSegmentationParameter::getShowTexts(vector<string>& showTexts) const {
  showTexts.push_back(prefix_ + RESOLUTION_OPT ": " + to_string(resolution));
  showTexts.push_back(prefix_ + K_OPT ": " + to_string(k));
  showTexts.push_back(prefix_ + ALPHA_OPT ": " + to_string(alpha));
  showTexts.push_back(prefix_ + NULL_ALPHA_RATIO_OPT ": " + to_string(nullAlphaRatio));
  showTexts.push_back(prefix_ + N_TRAIN_OPT ": " + to_string(nTrain));
  showTexts.push_back(prefix_ + DYNAMIC_THRESHOLD_OPT ": " + to_string(dynamicThresholdLE));
  showTexts.push_back(prefix_ + STATIC_THRESHOLD_OPT ": " + to_string(staticThresholdGT));
  showTexts.push_back(prefix_ + STATIC_THRESHOLD_OPT ": " + to_string(staticThresholdGT));
}

void JPCCSegmentationParameter::notify() {
  type            = getSegmentationType(type_);
  staticPointType = getStaticPointType(staticPointType_);
  assert(resolution > 0.0);
  assert(k > 0);
  assert(alpha > 0.0);
  assert((alpha * nullAlphaRatio) > 0.0);
  assert(nTrain > 0.0);
}

ostream& operator<<(ostream& out, const JPCCSegmentationParameter& obj) {
  obj.coutParameters(out)                                             //
      (TYPE_OPT, obj.type_)                                           //
      (STATIC_POINT_TYPE_OPT, obj.staticPointType_)                   //
      (UPDATE_MODEL_BEFORE_N_TRAIN_OPT, obj.updateModelBeforeNTrain)  //
      (RESOLUTION_OPT, obj.resolution)                                //
      (K_OPT, obj.k)                                                  //
      (ALPHA_OPT, obj.alpha)                                          //
      (NULL_ALPHA_RATIO_OPT, obj.nullAlphaRatio)                      //
      (N_TRAIN_OPT, obj.nTrain)                                       //
      (DYNAMIC_THRESHOLD_OPT, obj.dynamicThresholdLE)                 //
      (STATIC_THRESHOLD_OPT, obj.staticThresholdGT)                   //
      (MINIMUM_VARIANCE_OPT, obj.minimumVariance)                     //
      ;
  return out;
}

SegmentationType getSegmentationType(const string& segmentationType) {
  if (segmentationType == "gmm") {
    return SegmentationType::GMM;
  } else {
    throw logic_error("invalid segmentationType");
  }
}

StaticPointType getStaticPointType(const string& staticPointType) {
  if (staticPointType == "center") {
    return StaticPointType::CENTER;
  } else {
    throw logic_error("invalid staticPointType");
  }
}

}  // namespace jpcc::segmentation
