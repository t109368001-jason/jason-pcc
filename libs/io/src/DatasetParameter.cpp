#include <jpcc/io/DatasetParameter.h>

namespace jpcc::io {

using namespace std;
using namespace po;

#define DATASET_OPT_PREFIX "dataset"
#define NAME_OPT ".name"
#define SENSOR_OPT ".sensor"
#define TYPE_OPT ".type"
#define FOLDER_PREFIX_OPT ".folderPrefix"
#define FOLDER_OPT ".folder"
#define FILES_OPT ".files"
#define FRAME_COUNTS_OPT ".frameCounts"
#define START_FRAME_NUMBERS_OPT ".startFrameNumbers"
#define HAVE_GPS_TIME_OPT ".haveGpsTime"

DatasetParameter::DatasetParameter() : DatasetParameter(DATASET_OPT_PREFIX, __FUNCTION__) {}

DatasetParameter::DatasetParameter(const std::string& prefix, const std::string& caption) :
    Parameter(prefix, caption),
    folderPrefix("../../dataset/"),
    files(),
    frameCounts(),
    startFrameNumbers(),
    haveGpsTime(false) {
  opts_.add_options()                                                                                            //
      (string(prefix_ + NAME_OPT).c_str(), value<string>(&name), "name")                                         //
      (string(prefix_ + SENSOR_OPT).c_str(), value<string>(&sensor), "sensor")                                   //
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

void DatasetParameter::notify() {
  assert(!name.empty());
  assert(!type.empty());
  assert(!folder.empty());
  assert(frameCounts.size() == files.size());
  assert((startFrameNumbers.empty()) || (startFrameNumbers.size() == files.size()));
}

size_t DatasetParameter::count() const { return files.size(); }

std::string DatasetParameter::getFilePath(const size_t index) const { return folderPrefix + folder + files.at(index); }

size_t DatasetParameter::getFrameCounts(size_t index) const { return frameCounts.at(index); }

size_t DatasetParameter::getStartFrameNumbers(size_t index) const { return startFrameNumbers.at(index); }

ostream& operator<<(ostream& out, const DatasetParameter& obj) {
  out << obj.caption_ << endl;
  out << "\t" << obj.prefix_ << NAME_OPT "=" << obj.name << endl;
  out << "\t" << obj.prefix_ << SENSOR_OPT "=" << obj.sensor << endl;
  out << "\t" << obj.prefix_ << TYPE_OPT "=" << obj.type << endl;
  out << "\t" << obj.prefix_ << FOLDER_PREFIX_OPT "=" << obj.folderPrefix << endl;
  out << "\t" << obj.prefix_ << FOLDER_OPT "=" << obj.folder << endl;
  out << "\t" << obj.prefix_ << FILES_OPT "=";
  for (size_t i = 0; i < obj.files.size(); i++) {
    if (i == 0) { out << "["; }
    if (i != 0) { out << ", "; }
    out << obj.files.at(i);
    if (i == (obj.files.size() - 1)) { out << "]"; }
  }
  out << endl;
  out << "\t" << obj.prefix_ << FRAME_COUNTS_OPT "=";
  for (size_t i = 0; i < obj.frameCounts.size(); i++) {
    if (i == 0) { out << "["; }
    if (i != 0) { out << ", "; }
    out << obj.frameCounts.at(i);
    if (i == (obj.frameCounts.size() - 1)) { out << "]"; }
  }
  out << endl;
  out << "\t" << obj.prefix_ << START_FRAME_NUMBERS_OPT "=";
  for (size_t i = 0; i < obj.startFrameNumbers.size(); i++) {
    if (i == 0) { out << "["; }
    if (i != 0) { out << ", "; }
    out << obj.startFrameNumbers.at(i);
    if (i == (obj.startFrameNumbers.size() - 1)) { out << "]"; }
  }
  out << endl;
  out << "\t" << obj.prefix_ << HAVE_GPS_TIME_OPT "=" << obj.haveGpsTime << endl;
  return out;
}

}  // namespace jpcc::io
