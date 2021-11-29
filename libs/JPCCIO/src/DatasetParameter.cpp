#include <jpcc/io/DatasetParameter.h>

namespace jpcc {
namespace io {

DatasetParameter::DatasetParameter() : Parameter("DatasetOptions") {
  opts_.add_options()                                                                         //
      ("dataset.type", po::value<std::string>(&datasetType), "dataset type")                  //
      ("dataset.folder", po::value<std::string>(&datasetFolder), "dataset folder")            //
      ("dataset.files", po::value<std::vector<std::string>>(&datasetFiles), "dataset files")  //
      ;
}

std::vector<std::array<std::string, 2>> DatasetParameter::getDependencies() {
  return {
      {"dataset.folder", "dataset.type"},  //
      {"dataset.files", "dataset.type"},   //
      {"dataset.files", "dataset.folder"}  //
  };
}

std::ostream& operator<<(std::ostream& out, const DatasetParameter& obj) {
  out << "DatasetParameter" << std::endl;
  out << "\tdatasetType=" << obj.datasetType << std::endl;
  out << "\tdatasetFolder=" << obj.datasetFolder << std::endl;
  out << "\tdtasetFiles=";
  for (size_t i = 0; i < obj.datasetFiles.size(); i++) {
    if (i == 0) { out << "["; }
    if (i != 0) { out << ", "; }
    out << obj.datasetFiles[i];
    if (i == (obj.datasetFiles.size() - 1)) { out << "]" << std::endl; }
  }
  return out;
}

}  // namespace io
}  // namespace jpcc
