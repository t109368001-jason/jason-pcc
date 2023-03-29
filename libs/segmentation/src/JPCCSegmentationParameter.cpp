#include <jpcc/segmentation/JPCCSegmentationParameter.h>

namespace jpcc::segmentation {

using namespace std;
using namespace po;

#define TYPE_OPT ".type"
#define OUTPUT_TYPE_OPT ".outputType"
#define STATIC_POINT_TYPE_OPT ".staticPointType"
#define UPDATE_MODEL_BEFORE_N_TRAIN_OPT ".updateModelBeforeNTrain"
#define RESOLUTION_OPT ".resolution"
#define K_OPT ".k"
#define ALPHA_OPT ".alpha"
#define N_TRAIN_OPT ".nTrain"
#define STATIC_THRESHOLD1_OPT ".staticThreshold1"
#define STATIC_THRESHOLD2_OPT ".staticThreshold2"
#define NULL_STATIC_THRESHOLD1_OPT ".nullStaticThreshold1"
#define NULL_STATIC_THRESHOLD2_OPT ".nullStaticThreshold2"
#define MINIMUM_VARIANCE_OPT ".minimumVariance"

JPCCSegmentationParameter::JPCCSegmentationParameter() :
    JPCCSegmentationParameter(JPCC_GMM_SEGMENTATION_OPT_PREFIX, __FUNCTION__) {}

JPCCSegmentationParameter::JPCCSegmentationParameter(const string& prefix, const string& caption) :
    Parameter(prefix, caption),
    type_("none"),
    outputType_("none"),
    staticPointType_("center"),
    kVector_(),
    alphaVector_(),
    nTrainVector_(),
    staticThreshold1Vector_(),
    staticThreshold2Vector_(),
    nullStaticThreshold1Vector_(),
    nullStaticThreshold2Vector_(),
    type(SegmentationType::NONE),
    outputType(SegmentationOutputType::NONE),
    staticPointType(StaticPointType::CENTER),
    updateModelBeforeNTrain(false),
    resolution(100.0),
    minimumVariance(0.0016) {
  opts_.add_options()                                                //
      (string(prefix_ + TYPE_OPT).c_str(),                           //
       value<string>(&type_),                                        //
       "type")                                                       //
      (string(prefix_ + OUTPUT_TYPE_OPT).c_str(),                    //
       value<string>(&outputType_),                                  //
       "outputType")                                                 //
      (string(prefix_ + STATIC_POINT_TYPE_OPT).c_str(),              //
       value<string>(&staticPointType_),                             //
       "staticPointType, [center]")                                  //
      (string(prefix_ + UPDATE_MODEL_BEFORE_N_TRAIN_OPT).c_str(),    //
       value<bool>(&updateModelBeforeNTrain),                        //
       JPCC_GMM_SEGMENTATION_OPT_PREFIX " updateModelBeforeNTrain")  //
      (string(prefix_ + RESOLUTION_OPT).c_str(),                     //
       value<double>(&resolution),                                   //
       JPCC_GMM_SEGMENTATION_OPT_PREFIX " resolution")               //
      (string(prefix_ + K_OPT).c_str(),                              //
       value<vector<int>>(&kVector_),                                //
       JPCC_GMM_SEGMENTATION_OPT_PREFIX " k")                        //
      (string(prefix_ + ALPHA_OPT).c_str(),                          //
       value<vector<double>>(&alphaVector_),                         //
       JPCC_GMM_SEGMENTATION_OPT_PREFIX " alpha")                    //
      (string(prefix_ + N_TRAIN_OPT).c_str(),                        //
       value<vector<int>>(&nTrainVector_),                           //
       JPCC_GMM_SEGMENTATION_OPT_PREFIX " nTrain")                   //
      (string(prefix_ + STATIC_THRESHOLD1_OPT).c_str(),              //
       value<vector<double>>(&staticThreshold1Vector_),              //
       JPCC_GMM_SEGMENTATION_OPT_PREFIX " staticThreshold1")         //
      (string(prefix_ + STATIC_THRESHOLD2_OPT).c_str(),              //
       value<vector<double>>(&staticThreshold2Vector_),              //
       JPCC_GMM_SEGMENTATION_OPT_PREFIX " staticThreshold2")         //
      (string(prefix_ + NULL_STATIC_THRESHOLD1_OPT).c_str(),         //
       value<vector<double>>(&nullStaticThreshold1Vector_),          //
       JPCC_GMM_SEGMENTATION_OPT_PREFIX " nullStaticThreshold1")     //
      (string(prefix_ + NULL_STATIC_THRESHOLD2_OPT).c_str(),         //
       value<vector<double>>(&nullStaticThreshold2Vector_),          //
       JPCC_GMM_SEGMENTATION_OPT_PREFIX " nullStaticThreshold2")     //
      (string(prefix_ + MINIMUM_VARIANCE_OPT).c_str(),               //
       value<double>(&minimumVariance),                              //
       JPCC_GMM_SEGMENTATION_OPT_PREFIX " minimumVariance")          //
      ;
}

void JPCCSegmentationParameter::getShowTexts(vector<string>& showTexts) const {
  showTexts.push_back(prefix_ + RESOLUTION_OPT ": " + to_string(resolution));
  showTexts.push_back(prefix_ + K_OPT ": " + to_string(kVector_.front()));
  showTexts.push_back(prefix_ + ALPHA_OPT ": " + to_string(alphaVector_.front()));
  showTexts.push_back(prefix_ + N_TRAIN_OPT ": " + to_string(nTrainVector_.front()));
  showTexts.push_back(prefix_ + STATIC_THRESHOLD1_OPT ": " + to_string(staticThreshold1Vector_.front()));
  showTexts.push_back(prefix_ + STATIC_THRESHOLD2_OPT ": " + to_string(staticThreshold2Vector_.front()));
  showTexts.push_back(prefix_ + NULL_STATIC_THRESHOLD1_OPT ": " + to_string(nullStaticThreshold1Vector_.front()));
  showTexts.push_back(prefix_ + NULL_STATIC_THRESHOLD2_OPT ": " + to_string(nullStaticThreshold2Vector_.front()));
}

void JPCCSegmentationParameter::notify() {
  type            = getSegmentationType(type_);
  outputType      = getSegmentationOutputType(outputType_);
  staticPointType = getStaticPointType(staticPointType_);
  if (type == SegmentationType::NONE) { return; }
  assert(resolution > 0.0);
  assert(!kVector_.empty());
  assert(!alphaVector_.empty());
  assert(!nTrainVector_.empty());
  assert(!staticThreshold1Vector_.empty());
  assert(!staticThreshold2Vector_.empty());
  assert(!nullStaticThreshold1Vector_.empty());
  assert(!nullStaticThreshold2Vector_.empty());
  for (auto& k : kVector_) { assert(k > 0); }
  for (size_t i = 0; i < alphaVector_.size(); i++) { assert(alphaVector_[i] > 0.0); }
  for (auto& nTrain : nTrainVector_) { assert(nTrain > 0.0); }
}

int    JPCCSegmentationParameter::getK() const { return kVector_.front(); }
double JPCCSegmentationParameter::getAlpha() const { return alphaVector_.front(); }
int    JPCCSegmentationParameter::getNTrain() const { return nTrainVector_.front(); }
double JPCCSegmentationParameter::getStaticThreshold1() const { return staticThreshold1Vector_.front(); }
double JPCCSegmentationParameter::getStaticThreshold2() const { return staticThreshold2Vector_.front(); }
double JPCCSegmentationParameter::getNullStaticThreshold1() const { return nullStaticThreshold1Vector_.front(); }
double JPCCSegmentationParameter::getNullStaticThreshold2() const { return nullStaticThreshold2Vector_.front(); }

int    JPCCSegmentationParameter::getK(const int index) const { return kVector_[index]; }
double JPCCSegmentationParameter::getAlpha(const int index) const { return alphaVector_[index]; }
int    JPCCSegmentationParameter::getNTrain(const int index) const { return nTrainVector_[index]; }
double JPCCSegmentationParameter::getStaticThreshold1(const int index) const { return staticThreshold1Vector_[index]; }
double JPCCSegmentationParameter::getStaticThreshold2(const int index) const { return staticThreshold2Vector_[index]; }
double JPCCSegmentationParameter::getNullStaticThreshold1(const int index) const {
  return nullStaticThreshold1Vector_[index];
}
double JPCCSegmentationParameter::getNullStaticThreshold2(const int index) const {
  return nullStaticThreshold2Vector_[index];
}

const std::vector<int>&    JPCCSegmentationParameter::getKVector() const { return kVector_; }
const std::vector<double>& JPCCSegmentationParameter::getAlphaVector() const { return alphaVector_; }
const std::vector<int>&    JPCCSegmentationParameter::getNTrainVector() const { return nTrainVector_; }
const std::vector<double>& JPCCSegmentationParameter::getStaticThreshold1Vector() const {
  return staticThreshold1Vector_;
}
const std::vector<double>& JPCCSegmentationParameter::getStaticThreshold2Vector() const {
  return staticThreshold2Vector_;
}
const std::vector<double>& JPCCSegmentationParameter::getNullStaticThreshold1Vector() const {
  return nullStaticThreshold1Vector_;
}
const std::vector<double>& JPCCSegmentationParameter::getNullStaticThreshold2Vector() const {
  return nullStaticThreshold2Vector_;
}

ostream& operator<<(ostream& out, const JPCCSegmentationParameter& obj) {
  obj.coutParameters(out)                                             //
      (TYPE_OPT, obj.type_)                                           //
      (OUTPUT_TYPE_OPT, obj.outputType_)                              //
      (STATIC_POINT_TYPE_OPT, obj.staticPointType_)                   //
      (UPDATE_MODEL_BEFORE_N_TRAIN_OPT, obj.updateModelBeforeNTrain)  //
      (RESOLUTION_OPT, obj.resolution)                                //
      (K_OPT, obj.kVector_)                                           //
      (ALPHA_OPT, obj.alphaVector_)                                   //
      (N_TRAIN_OPT, obj.nTrainVector_)                                //
      (STATIC_THRESHOLD1_OPT, obj.staticThreshold1Vector_)            //
      (STATIC_THRESHOLD2_OPT, obj.staticThreshold2Vector_)            //
      (NULL_STATIC_THRESHOLD1_OPT, obj.nullStaticThreshold1Vector_)   //
      (NULL_STATIC_THRESHOLD2_OPT, obj.nullStaticThreshold2Vector_)   //
      (MINIMUM_VARIANCE_OPT, obj.minimumVariance)                     //
      ;
  return out;
}

}  // namespace jpcc::segmentation
