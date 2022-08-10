#include <jpcc/segmentation/JPCCSegmentationParameter.h>

namespace jpcc::segmentation {

using namespace std;
using namespace po;

#define TYPE_OPT ".type"
#define STATIC_POINT_TYPE_OPT ".staticPointType"
#define UPDATE_MODEL_BEFORE_N_TRAIN_OPT ".updateModelBeforeNTrain"
#define OUTPUT_EXISTS_POINT_ONLY_OPT ".outputExistsPointOnly"
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
    Parameter(prefix, caption), type_(), staticPointType_(), updateModelBeforeNTrain(false) {
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
      (string(prefix_ + OUTPUT_EXISTS_POINT_ONLY_OPT).c_str(),       //
       value<vector<bool>>(&outputExistsPointOnlyVector_),           //
       JPCC_GMM_SEGMENTATION_OPT_PREFIX " outputExistsPointOnly")    //
      (string(prefix_ + RESOLUTION_OPT).c_str(),                     //
       value<double>(&resolution)->required(),                       //
       JPCC_GMM_SEGMENTATION_OPT_PREFIX " resolution")               //
      (string(prefix_ + K_OPT).c_str(),                              //
       value<vector<int>>(&kVector_)->required(),                    //
       JPCC_GMM_SEGMENTATION_OPT_PREFIX " k")                        //
      (string(prefix_ + ALPHA_OPT).c_str(),                          //
       value<vector<double>>(&alphaVector_)->required(),             //
       JPCC_GMM_SEGMENTATION_OPT_PREFIX " alpha")                    //
      (string(prefix_ + NULL_ALPHA_RATIO_OPT).c_str(),               //
       value<vector<double>>(&nullAlphaRatioVector_)->required(),    //
       JPCC_GMM_SEGMENTATION_OPT_PREFIX " nullAlphaRatio")           //
      (string(prefix_ + N_TRAIN_OPT).c_str(),                        //
       value<vector<int>>(&nTrainVector_)->required(),               //
       JPCC_GMM_SEGMENTATION_OPT_PREFIX " nTrain")                   //
      (string(prefix_ + DYNAMIC_THRESHOLD_OPT).c_str(),              //
       value<vector<double>>(&dynamicThresholdLEVector_),            //
       JPCC_GMM_SEGMENTATION_OPT_PREFIX " dynamicThresholdLE")       //
      (string(prefix_ + STATIC_THRESHOLD_OPT).c_str(),               //
       value<vector<double>>(&staticThresholdGTVector_),             //
       JPCC_GMM_SEGMENTATION_OPT_PREFIX " staticThresholdGT")        //
      (string(prefix_ + MINIMUM_VARIANCE_OPT).c_str(),               //
       value<double>(&minimumVariance)->required(),                  //
       JPCC_GMM_SEGMENTATION_OPT_PREFIX " minimumVariance")          //
      ;
}

void JPCCSegmentationParameter::getShowTexts(vector<string>& showTexts) const {
  showTexts.push_back(prefix_ + RESOLUTION_OPT ": " + to_string(resolution));
  showTexts.push_back(prefix_ + K_OPT ": " + to_string(kVector_.at(0)));
  showTexts.push_back(prefix_ + ALPHA_OPT ": " + to_string(alphaVector_.at(0)));
  showTexts.push_back(prefix_ + NULL_ALPHA_RATIO_OPT ": " + to_string(nullAlphaRatioVector_.at(0)));
  showTexts.push_back(prefix_ + N_TRAIN_OPT ": " + to_string(nTrainVector_.at(0)));
  showTexts.push_back(prefix_ + DYNAMIC_THRESHOLD_OPT ": " + to_string(dynamicThresholdLEVector_.at(0)));
  showTexts.push_back(prefix_ + STATIC_THRESHOLD_OPT ": " + to_string(staticThresholdGTVector_.at(0)));
  showTexts.push_back(prefix_ + STATIC_THRESHOLD_OPT ": " + to_string(staticThresholdGTVector_.at(0)));
}

void JPCCSegmentationParameter::notify() {
  type            = getSegmentationType(type_);
  staticPointType = getStaticPointType(staticPointType_);
  assert(resolution > 0.0);
  assert(!outputExistsPointOnlyVector_.empty());
  assert(!kVector_.empty());
  assert(!alphaVector_.empty());
  assert(!nullAlphaRatioVector_.empty());
  assert(!nTrainVector_.empty());
  assert(!dynamicThresholdLEVector_.empty());
  assert(!staticThresholdGTVector_.empty());
  for (auto& k : kVector_) { assert(k > 0); }
  for (size_t i = 0; i < alphaVector_.size(); i++) {
    assert(alphaVector_.at(i) > 0.0);
    assert((alphaVector_.at(i) * nullAlphaRatioVector_.at(i)) > 0.0);
  }
  for (auto& nTrain : nTrainVector_) { assert(nTrain > 0.0); }
}

bool   JPCCSegmentationParameter::getOutputExistsPointOnly() const { return outputExistsPointOnlyVector_.at(0); }
int    JPCCSegmentationParameter::getK() const { return kVector_.at(0); }
double JPCCSegmentationParameter::getAlpha() const { return alphaVector_.at(0); }
double JPCCSegmentationParameter::getNullAlphaRatio() const { return nullAlphaRatioVector_.at(0); }
int    JPCCSegmentationParameter::getNTrain() const { return nTrainVector_.at(0); }
double JPCCSegmentationParameter::getDynamicThresholdLE() const { return dynamicThresholdLEVector_.at(0); }
double JPCCSegmentationParameter::getStaticThresholdGT() const { return staticThresholdGTVector_.at(0); }

bool JPCCSegmentationParameter::getOutputExistsPointOnly(const int index) const {
  return outputExistsPointOnlyVector_.at(index);
}
int    JPCCSegmentationParameter::getK(const int index) const { return kVector_.at(index); }
double JPCCSegmentationParameter::getAlpha(const int index) const { return alphaVector_.at(index); }
double JPCCSegmentationParameter::getNullAlphaRatio(const int index) const { return nullAlphaRatioVector_.at(index); }
int    JPCCSegmentationParameter::getNTrain(const int index) const { return nTrainVector_.at(index); }
double JPCCSegmentationParameter::getDynamicThresholdLE(const int index) const {
  return dynamicThresholdLEVector_.at(index);
}
double JPCCSegmentationParameter::getStaticThresholdGT(const int index) const {
  return staticThresholdGTVector_.at(index);
}

const std::vector<bool>& JPCCSegmentationParameter::getOutputExistsPointOnlyVector() const {
  return outputExistsPointOnlyVector_;
}
const std::vector<int>&    JPCCSegmentationParameter::getKVector() const { return kVector_; }
const std::vector<double>& JPCCSegmentationParameter::getAlphaVector() const { return alphaVector_; }
const std::vector<double>& JPCCSegmentationParameter::getNullAlphaRatioVector() const { return nullAlphaRatioVector_; }
const std::vector<int>&    JPCCSegmentationParameter::getNTrainVector() const { return nTrainVector_; }
const std::vector<double>& JPCCSegmentationParameter::getDynamicThresholdLEVector() const {
  return dynamicThresholdLEVector_;
}
const std::vector<double>& JPCCSegmentationParameter::getStaticThresholdGTVector() const {
  return staticThresholdGTVector_;
}

ostream& operator<<(ostream& out, const JPCCSegmentationParameter& obj) {
  obj.coutParameters(out)                                               //
      (TYPE_OPT, obj.type_)                                             //
      (STATIC_POINT_TYPE_OPT, obj.staticPointType_)                     //
      (UPDATE_MODEL_BEFORE_N_TRAIN_OPT, obj.updateModelBeforeNTrain)    //
      (OUTPUT_EXISTS_POINT_ONLY_OPT, obj.outputExistsPointOnlyVector_)  //
      (RESOLUTION_OPT, obj.resolution)                                  //
      (K_OPT, obj.kVector_)                                             //
      (ALPHA_OPT, obj.alphaVector_)                                     //
      (NULL_ALPHA_RATIO_OPT, obj.nullAlphaRatioVector_)                 //
      (N_TRAIN_OPT, obj.nTrainVector_)                                  //
      (DYNAMIC_THRESHOLD_OPT, obj.dynamicThresholdLEVector_)            //
      (STATIC_THRESHOLD_OPT, obj.staticThresholdGTVector_)              //
      (MINIMUM_VARIANCE_OPT, obj.minimumVariance)                       //
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
