#include <jpcc/io/DatasetParameter.h>

#include <exception>

namespace jpcc {
namespace io {

using namespace std;
using namespace po;

#define DATASET_OPT_PREFIX "dataset"
#define SENSOR_OPT DATASET_OPT_PREFIX ".sensor"
#define TYPE_OPT DATASET_OPT_PREFIX ".type"
#define FOLDER_OPT DATASET_OPT_PREFIX ".folder"
#define TOTAL_FILES_OPT DATASET_OPT_PREFIX ".totalFiles"
#define FILES_OPT DATASET_OPT_PREFIX ".files"
#define FRAME_COUNTS_OPT DATASET_OPT_PREFIX ".frameCounts"
#define HAVE_GPS_TIME_OPT DATASET_OPT_PREFIX ".haveGpsTime"

DatasetParameter::DatasetParameter() :
    sensor(""), type(""), folder(""), totalFiles(0), files(), frameCounts(), haveGpsTime(false) {}

options_description DatasetParameter::getOpts() {
  options_description opts("DatasetOptions");
  opts.add_options()                                                                               //
      (SENSOR_OPT, value<string>(&sensor)->required(), "sensor")                                   //
      (TYPE_OPT, value<string>(&type)->required(), "dataset type")                                 //
      (FOLDER_OPT, value<string>(&folder)->required(), "dataset folder")                           //
      (TOTAL_FILES_OPT, value<size_t>(&totalFiles)->required(), "dataset folder")                  //
      (FILES_OPT, value<vector<string>>(&files), "dataset files")                                  //
      (FRAME_COUNTS_OPT, value<vector<size_t>>(&frameCounts), "dataset files")                     //
      (HAVE_GPS_TIME_OPT, value<bool>(&haveGpsTime)->default_value(haveGpsTime), "have gps time")  //
      ;
  return opts;
}

vector<array<string, 2>> DatasetParameter::getDependencies() {
  return {
      {FILES_OPT, TOTAL_FILES_OPT},   //
      {FRAME_COUNTS_OPT, FILES_OPT},  //
  };
}

void DatasetParameter::notify() {
  if (totalFiles != files.size()) { throw runtime_error(TOTAL_FILES_OPT " not match number of " FILES_OPT); }
  if (totalFiles != frameCounts.size()) {
    throw runtime_error(TOTAL_FILES_OPT " not match number of " FRAME_COUNTS_OPT);
  }
}

std::string DatasetParameter::getFilePath(const size_t index) const {
  assert(index < files.size());
  return folder + files[index];
}

ostream& operator<<(ostream& out, const DatasetParameter& obj) {
  out << "DatasetParameter" << endl;
  out << "\t" TYPE_OPT "=" << obj.type << endl;
  out << "\t" FOLDER_OPT "=" << obj.folder << endl;
  out << "\t" TOTAL_FILES_OPT "=" << obj.totalFiles << endl;
  out << "\t" FILES_OPT "=";
  for (size_t i = 0; i < obj.files.size(); i++) {
    if (i == 0) { out << "["; }
    if (i != 0) { out << ", "; }
    out << obj.files[i];
    if (i == (obj.files.size() - 1)) { out << "]" << endl; }
  }
  out << "\t" FRAME_COUNTS_OPT "=";
  for (size_t i = 0; i < obj.frameCounts.size(); i++) {
    if (i == 0) { out << "["; }
    if (i != 0) { out << ", "; }
    out << obj.frameCounts[i];
    if (i == (obj.frameCounts.size() - 1)) { out << "]" << endl; }
  }
  out << "\t" HAVE_GPS_TIME_OPT "=" << obj.haveGpsTime << endl;
  return out;
}

}  // namespace io
}  // namespace jpcc
