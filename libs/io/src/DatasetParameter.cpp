#include <jpcc/io/DatasetParameter.h>

#include <boost/algorithm/string.hpp>

namespace jpcc::io {

using namespace std;
using namespace std::filesystem;
using namespace po;
using namespace Eigen;

#define DATASET_OPT_PREFIX "dataset"
#define NAME_OPT ".name"
#define SENSOR_OPT ".sensor"
#define TYPE_OPT ".type"
#define PRE_PROCESSED_OPT ".preProcessed"
#define FOLDER_PREFIX_OPT ".folderPrefix"
#define FOLDER_OPT ".folder"
#define FILES_OPT ".files"
#define FRAME_COUNTS_OPT ".frameCounts"
#define START_FRAME_NUMBERS_OPT ".startFrameNumbers"
#define TRANSFORMS_OPT ".transforms"
#define HAVE_GPS_TIME_OPT ".haveGpsTime"

DatasetParameter::DatasetParameter() : DatasetParameter(DATASET_OPT_PREFIX, __FUNCTION__) {}

DatasetParameter::DatasetParameter(const string& prefix, const string& caption) :
    Parameter(prefix, caption),
    preProcessed(false),
    folderPrefix("../../dataset/"),
    files(),
    frameCounts(),
    startFrameNumbers(),
    transforms(),
    haveGpsTime(false) {
  opts_.add_options()                                                                                            //
      (string(prefix_ + NAME_OPT).c_str(), value<string>(&name), "name")                                         //
      (string(prefix_ + SENSOR_OPT).c_str(), value<string>(&sensor), "sensor")                                   //
      (string(prefix_ + PRE_PROCESSED_OPT).c_str(),                                                              //
       value<bool>(&preProcessed)->default_value(preProcessed),                                                  //
       "preProcessed")                                                                                           //
      (string(prefix_ + TYPE_OPT).c_str(), value<string>(&type), "dataset type")                                 //
      (string(prefix_ + FOLDER_PREFIX_OPT).c_str(),                                                              //
       value<string>(&folderPrefix)->default_value(folderPrefix),                                                //
       "dataset folder prefix")                                                                                  //
      (string(prefix_ + FOLDER_OPT).c_str(), value<string>(&folder), "dataset folder")                           //
      (string(prefix_ + FILES_OPT).c_str(), value<vector<string>>(&files), "dataset files")                      //
      (string(prefix_ + FRAME_COUNTS_OPT).c_str(), value<vector<size_t>>(&frameCounts), "dataset frame counts")  //
      (string(prefix_ + START_FRAME_NUMBERS_OPT).c_str(),                                                        //
       value<vector<size_t>>(&startFrameNumbers),                                                                //
       "dataset start frame numbers")                                                                            //
      (string(prefix_ + TRANSFORMS_OPT).c_str(), value<vector<string>>(&transforms_), "dataset transforms")      //
      (string(prefix_ + HAVE_GPS_TIME_OPT).c_str(),                                                              //
       value<bool>(&haveGpsTime)->default_value(haveGpsTime),                                                    //
       "have gps time")                                                                                          //
      ;
}

vector<array<string, 2>> DatasetParameter::getDependencies() const {
  return {
      {FRAME_COUNTS_OPT, FILES_OPT},  //
  };
}

void DatasetParameter::getShowTexts(vector<string>& showTexts) const {
  showTexts.push_back(prefix_ + NAME_OPT ": " + name);
  showTexts.push_back(prefix_ + FRAME_COUNTS_OPT ": " + to_string(getFrameCount()));
}

void DatasetParameter::notify() { notify(true); }

void DatasetParameter::notify(bool isInput) {
  ASSERT_THROW(!name.empty());
  ASSERT_THROW(!type.empty());
  if (preProcessed) { ASSERT_THROW(type == "ply"); }
  ASSERT_THROW(!folderPrefix.empty());
  ASSERT_THROW(!folder.empty());
  ASSERT_THROW(frameCounts.size() == files.size());
  ASSERT_THROW((startFrameNumbers.empty()) || (startFrameNumbers.size() == files.size()));
  endFrameNumbers.resize(startFrameNumbers.size());
  for (size_t i = 0; i < startFrameNumbers.size(); i++) {
    endFrameNumbers.at(i) = startFrameNumbers.at(i) + frameCounts.at(i);
  }
  folderPath = path(folderPrefix) / folder;
  filePaths.resize(files.size());
  for (size_t i = 0; i < files.size(); i++) {
    filePaths.at(i) = folderPath / files.at(i);
    if (isInput) {
      if (type == "ply") {
        char fileName[4096];
        sprintf(fileName, filePaths.at(i).string().c_str(), startFrameNumbers.at(i));
        ASSERT_THROW(exists(path(fileName)));
      } else {
        ASSERT_THROW(exists(filePaths.at(i)));
      }
    }
  }
  if (!transforms_.empty()) {
    ASSERT_THROW(type != "ply");
    ASSERT_THROW(transforms_.size() == files.size());
    transforms.resize(transforms_.size());
    for (size_t i = 0; i < transforms_.size(); i++) {
      transforms.at(i) = jpcc::make_shared<Matrix4f>();
      vector<string> ss;
      boost::algorithm::split(ss, transforms_.at(i), boost::is_any_of(","));
      ASSERT_THROW(ss.size() == (*transforms.at(i)).size());
      auto it = ss.begin();
      for (int ii = 0; ii < 4; ii++) {
        for (int jj = 0; jj < 4; jj++) { (*transforms.at(i))(ii, jj) = stof(*it++); }
      }
    }
  }
}

size_t DatasetParameter::count() const { return files.size(); }

string DatasetParameter::getFilePath(const size_t index) const { return filePaths.at(index).string(); }

size_t DatasetParameter::getFrameCounts(const size_t index) const { return frameCounts.at(index); }

size_t DatasetParameter::getFrameCount() const { return *min_element(frameCounts.begin(), frameCounts.end()); }

size_t DatasetParameter::getStartFrameNumbers(const size_t index) const { return startFrameNumbers.at(index); }

size_t DatasetParameter::getStartFrameNumber() const {
  return *max_element(startFrameNumbers.begin(), startFrameNumbers.end());
}

size_t DatasetParameter::getEndFrameNumbers(const size_t index) const { return endFrameNumbers.at(index); }

size_t DatasetParameter::getEndFrameNumber() const {
  return *min_element(endFrameNumbers.begin(), endFrameNumbers.end());
}

shared_ptr<Matrix4f> DatasetParameter::getTransforms(const size_t index) const {
  return transforms.empty() ? nullptr : transforms.at(index);
}

ostream& operator<<(ostream& out, const DatasetParameter& obj) {
  obj.coutParameters(out)                               //
      (NAME_OPT, obj.name)                              //
      (SENSOR_OPT, obj.sensor)                          //
      (TYPE_OPT, obj.type)                              //
      (FOLDER_PREFIX_OPT, obj.folderPrefix)             //
      (FOLDER_OPT, obj.folder)                          //
      (FILES_OPT, obj.files)                            //
      (FRAME_COUNTS_OPT, obj.frameCounts)               //
      (START_FRAME_NUMBERS_OPT, obj.startFrameNumbers)  //
      (TRANSFORMS_OPT, obj.transforms)                  //
      (HAVE_GPS_TIME_OPT, obj.haveGpsTime)              //
      ;
  return out;
}

}  // namespace jpcc::io
