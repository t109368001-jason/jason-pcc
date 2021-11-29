#include <jpcc/io/DatasetParameter.h>

#include <exception>

namespace jpcc {
namespace io {

using namespace std;
using namespace po;

#define DATASET_OPT_PREFIX "dataset"
#define TYPE_OPT DATASET_OPT_PREFIX ".type"
#define FOLDER_OPT DATASET_OPT_PREFIX ".folder"
#define TOTAL_FILES_OPT DATASET_OPT_PREFIX ".totalFiles"
#define FILES_OPT DATASET_OPT_PREFIX ".files"
#define FRAME_COUNTS_OPT DATASET_OPT_PREFIX ".frameCounts"

DatasetParameter::DatasetParameter() : Parameter("DatasetOptions") {
  opts_.add_options()                                                           //
      (TYPE_OPT, value<string>(&type), "dataset type")                          //
      (FOLDER_OPT, value<string>(&folder), "dataset folder")                    //
      (TOTAL_FILES_OPT, value<size_t>(&totalFiles), "dataset folder")           //
      (FILES_OPT, value<vector<string>>(&files), "dataset files")               //
      (FRAME_COUNTS_OPT, value<vector<size_t>>(&frameCounts), "dataset files")  //
      ;
}

vector<array<string, 2>> DatasetParameter::getDependencies() {
  return {
      {FOLDER_OPT, TYPE_OPT},         //
      {TOTAL_FILES_OPT, FOLDER_OPT},  //
      {FILES_OPT, TYPE_OPT},          //
      {FILES_OPT, FOLDER_OPT},        //
      {FILES_OPT, TOTAL_FILES_OPT},   //
      {FRAME_COUNTS_OPT, FILES_OPT},  //
  };
}

void DatasetParameter::check() const {
  if (totalFiles != files.size()) { throw runtime_error(TOTAL_FILES_OPT " not match number of " FILES_OPT); }
  if (totalFiles != frameCounts.size()) {
    throw runtime_error(TOTAL_FILES_OPT " not match number of " FRAME_COUNTS_OPT);
  }
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
  return out;
}

}  // namespace io
}  // namespace jpcc
