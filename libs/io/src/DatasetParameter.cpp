#include <jpcc/io/DatasetParameter.h>

namespace jpcc::io {

using namespace std;
using namespace po;

#define DATASET_OPT_PREFIX "dataset"
#define SENSOR_OPT DATASET_OPT_PREFIX ".sensor"
#define TYPE_OPT DATASET_OPT_PREFIX ".type"
#define FOLDER_PREFIX_OPT DATASET_OPT_PREFIX ".folderPrefix"
#define FOLDER_OPT DATASET_OPT_PREFIX ".folder"
#define TOTAL_FILES_OPT DATASET_OPT_PREFIX ".totalFiles"
#define FILES_OPT DATASET_OPT_PREFIX ".files"
#define FRAME_COUNTS_OPT DATASET_OPT_PREFIX ".frameCounts"
#define HAVE_GPS_TIME_OPT DATASET_OPT_PREFIX ".haveGpsTime"

DatasetParameter::DatasetParameter() : DatasetParameter(DATASET_OPT_PREFIX, "DatasetOptions") {}

DatasetParameter::DatasetParameter(const std::string& prefix, const std::string& caption) :
    Parameter(prefix, caption),
    folderPrefix("../../dataset/"),
    totalFiles(0),
    files(),
    frameCounts(),
    haveGpsTime(false) {
  opts_.add_options()                                                                                          //
      (SENSOR_OPT, value<string>(&sensor), "sensor")                                                           //
      (TYPE_OPT, value<string>(&type), "dataset type")                                                         //
      (FOLDER_PREFIX_OPT, value<string>(&folderPrefix)->default_value(folderPrefix), "dataset folder prefix")  //
      (FOLDER_OPT, value<string>(&folder), "dataset folder")                                                   //
      (TOTAL_FILES_OPT, value<size_t>(&totalFiles), "dataset total files count")                               //
      (FILES_OPT, value<vector<string>>(&files), "dataset files")                                              //
      (FRAME_COUNTS_OPT, value<vector<size_t>>(&frameCounts), "dataset frame counts")                          //
      (HAVE_GPS_TIME_OPT, value<bool>(&haveGpsTime)->default_value(haveGpsTime), "have gps time")              //
      ;
}

vector<array<string, 2>> DatasetParameter::getDependencies() const {
  return {
      {FILES_OPT, TOTAL_FILES_OPT},   //
      {FRAME_COUNTS_OPT, FILES_OPT},  //
  };
}

void DatasetParameter::notify() {
  assert(!sensor.empty());
  assert(!type.empty());
  assert(!folder.empty());
  assert(totalFiles != 0);
  assert(totalFiles == files.size());
  assert(totalFiles == frameCounts.size());
}

std::string DatasetParameter::getFilePath(const size_t index) const { return folderPrefix + folder + files.at(index); }

ostream& operator<<(ostream& out, const DatasetParameter& obj) {
  out << obj.caption_ << endl;
  out << "\t" TYPE_OPT "=" << obj.type << endl;
  out << "\t" FOLDER_PREFIX_OPT "=" << obj.folder << endl;
  out << "\t" FOLDER_OPT "=" << obj.folder << endl;
  out << "\t" TOTAL_FILES_OPT "=" << obj.totalFiles << endl;
  out << "\t" FILES_OPT "=";
  for (size_t i = 0; i < obj.files.size(); i++) {
    if (i == 0) { out << "["; }
    if (i != 0) { out << ", "; }
    out << obj.files.at(i);
    if (i == (obj.files.size() - 1)) { out << "]"; }
  }
  out << endl;
  out << "\t" FRAME_COUNTS_OPT "=";
  for (size_t i = 0; i < obj.frameCounts.size(); i++) {
    if (i == 0) { out << "["; }
    if (i != 0) { out << ", "; }
    out << obj.frameCounts.at(i);
    if (i == (obj.frameCounts.size() - 1)) { out << "]"; }
  }
  out << endl;
  out << "\t" HAVE_GPS_TIME_OPT "=" << obj.haveGpsTime << endl;
  return out;
}

}  // namespace jpcc::io
